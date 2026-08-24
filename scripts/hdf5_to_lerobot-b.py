"""
Script to convert Aloha hdf5 data to the LeRobot dataset v2.0 format.

Example usage: uv run examples/aloha_real/convert_aloha_data_to_lerobot.py --raw-dir /path/to/raw/data --repo-id <org>/<dataset-name>
"""

import dataclasses
from pathlib import Path
import shutil
from typing import Literal

import h5py
# from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset
# from lerobot.common.datasets.push_dataset_to_hub._download_raw import download_raw
import numpy as np
import torch
import tqdm
import tyro
import cv2
import os
import argparse
import math
import yaml
import glob
from random import choice


@dataclasses.dataclass(frozen=True)
class DatasetConfig:
    use_videos: bool = True
    tolerance_s: float = 0.0001
    image_writer_processes: int = 8
    image_writer_threads: int = 8
    video_backend: str | None = None


DEFAULT_DATASET_CONFIG = DatasetConfig()


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    """Create 4x4 homogeneous transformation matrix from xyzrpy."""
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


def matrix_to_xyzrpy(matrix):
    """Extract xyzrpy from 4x4 transformation matrix."""
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def transform_pose_to_world(pose_base: np.ndarray, base_in_world: np.ndarray) -> np.ndarray:
    """Transform a pose from base frame to world frame.
    
    Args:
        pose_base: xyzrpy pose in base frame (6 elements)
        base_in_world: xyzrpy pose of base frame in world frame (6 elements)
    
    Returns:
        xyzrpy pose in world frame (6 elements)
    """
    # Create transformation matrices
    T_base_in_world = create_transformation_matrix(
        base_in_world[0], base_in_world[1], base_in_world[2],
        base_in_world[3], base_in_world[4], base_in_world[5]
    )
    T_pose_in_base = create_transformation_matrix(
        pose_base[0], pose_base[1], pose_base[2],
        pose_base[3], pose_base[4], pose_base[5]
    )
    
    # Transform: T_pose_in_world = T_base_in_world * T_pose_in_base
    T_pose_in_world = np.dot(T_base_in_world, T_pose_in_base)
    
    # Extract xyzrpy from result
    return np.array(matrix_to_xyzrpy(T_pose_in_world), dtype=np.float32)


def pose6d_to_matrix(pose: np.ndarray, use_row_pose6d: bool = False) -> np.ndarray:
    """
    将6D位姿转回4x4齐次变换矩阵
    
    Args:
        pose: [9,] 位姿向量 (3个平移 + 6个旋转)
              格式取决于 use_row_pose6d:
              - True:  [x, y, z, r00, r01, r10, r11, r20, r21] (按行)
              - False: [x, y, z, r00, r10, r20, r01, r11, r21] (按列)
        
    Returns:
        matrix: [4, 4] 齐次变换矩阵
    """
    assert pose.shape == (9,), "位姿必须是9维向量"
    
    # 1. 提取平移和旋转6D
    translation = pose[:3]  # [x, y, z]
    rot6d_flat = pose[3:]  # [6,]
    
    # 2. 根据格式重建旋转矩阵
    # 注意：无论哪种格式，都需要重建为两个3D列向量，然后叉乘得到第三列
    if use_row_pose6d:
        # 按行展平格式: [r00, r01, r10, r11, r20, r21]
        # 先重组为 (3, 2)，然后按列提取
        rot6d = rot6d_flat.reshape(3, 2)  # [[r00, r01], [r10, r11], [r20, r21]]
        col0 = rot6d[:, 0]  # [r00, r10, r20] - 第一列
        col1 = rot6d[:, 1]  # [r01, r11, r21] - 第二列
    else:
        # 按列展平格式: [r00, r10, r20, r01, r11, r21]
        # 先重组为 (2, 3)，然后转置再按列提取
        rot6d = rot6d_flat.reshape(2, 3)  # [[r00, r10, r20], [r01, r11, r21]]
        col0 = rot6d[0, :]  # [r00, r10, r20] - 第一列
        col1 = rot6d[1, :]  # [r01, r11, r21] - 第二列
    
    # 通过叉乘计算第三列（对两种格式都一样）
    col2 = np.cross(col0, col1)
    
    # 确保正交性（数值稳定性）
    col1 = np.cross(col2, col0)  # 重新正交化
    
    # 归一化
    col0 = col0 / np.linalg.norm(col0)
    col1 = col1 / np.linalg.norm(col1)
    col2 = col2 / np.linalg.norm(col2)
    
    # 组装旋转矩阵
    rotation_matrix = np.stack([col0, col1, col2], axis=1)  # [3, 3]
    
    # 3. 构建4x4矩阵
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = translation
    
    return matrix

def matrix_to_pose6d(matrix: np.ndarray, use_row_pose6d: bool = False) -> np.ndarray:
    """
    从4x4齐次变换矩阵提取6D位姿
    
    Args:
        matrix: [4, 4] 齐次变换矩阵
        
    Returns:
        pose: [9,] 6D位姿向量 (3个平移 + 6个旋转)
              格式取决于 use_row_pose6d:
              - True:  [x, y, z, r00, r01, r10, r11, r20, r21] (旋转矩阵前两行)
              - False: [x, y, z, r00, r10, r20, r01, r11, r21] (旋转矩阵前两列)
    """
    # 1. 提取平移
    translation = matrix[:3, 3]  # [x, y, z]
    
    # 2. 提取旋转矩阵并转换为6D表示
    rotation_matrix = matrix[:3, :3]
    
    if use_row_pose6d:
        # 按行取: 前两列按行展平
        # [[r00, r01, r02],     取前两列 →  [[r00, r01],
        #  [r10, r11, r12],                   [r10, r11],
        #  [r20, r21, r22]]                   [r20, r21]]
        # 按行展平 → [r00, r01, r10, r11, r20, r21]
        rotation_6d = rotation_matrix[:, :2].reshape(-1)  # [6,]
    else:
        # 按列取: 前两列按列展平
        # [[r00, r01, r02],     取前两列 →  [[r00, r01],
        #  [r10, r11, r12],                   [r10, r11],
        #  [r20, r21, r22]]                   [r20, r21]]
        # 转置后按行展平 → [r00, r10, r20, r01, r11, r21]
        rotation_6d = rotation_matrix[:, :2].T.reshape(-1)  # [6,]
    
    # 3. 组合
    pose_6d = np.concatenate([translation, rotation_6d])  # [9,]
    return pose_6d


def xyzrpy_to_pose6d(xyzrpy: np.ndarray, use_row_pose6d: bool = False) -> np.ndarray:
    """
    Convert xyzrpy pose to 6D pose representation.
    
    Args:
        xyzrpy: [6,] pose vector [x, y, z, roll, pitch, yaw]
        use_row_pose6d: If True, use row-major format [x, y, z, r00, r01, r10, r11, r20, r21]
                       If False (default), use column-major format [x, y, z, r00, r10, r20, r01, r11, r21]
        
    Returns:
        pose_6d: [9,] 6D pose vector
                 If use_row_pose6d=True: [x, y, z, r00, r01, r10, r11, r20, r21]
                 If use_row_pose6d=False: [x, y, z, r00, r10, r20, r01, r11, r21]
    """
    x, y, z, roll, pitch, yaw = xyzrpy
    matrix = create_transformation_matrix(x, y, z, roll, pitch, yaw)
    return matrix_to_pose6d(matrix, use_row_pose6d=use_row_pose6d)


def convert_poses_to_world(vec: list | np.ndarray, arm_base_in_world_list: list[np.ndarray]) -> list[float]:
    """Convert arm poses from base frame to world frame.
    
    Args:
        vec: Pose data vector. Each arm has 7 dimensions (xyzrpy 6D + gripper 1D).
             For N arms, total dimension is 7*N.
        arm_base_in_world_list: List of base poses in world frame for each arm.
                                Each element is a 6D xyzrpy vector.
    
    Returns:
        Converted pose vector with same dimensions, but poses transformed to world frame.
    """
    data = np.asarray(vec, dtype=np.float32)
    
    # Each arm has 7 dimensions (xyzrpy 6D + gripper 1D)
    arm_dim = 7
    num_arms = len(data) // arm_dim
    
    if len(data) % arm_dim != 0:
        raise ValueError(f"Data dimension {len(data)} is not a multiple of {arm_dim} (expected 7*N for N arms)")
    
    if len(arm_base_in_world_list) != num_arms:
        raise ValueError(f"Number of base poses ({len(arm_base_in_world_list)}) doesn't match number of arms ({num_arms})")
    
    result = data.copy()
    
    # Process each arm
    for arm_idx in range(num_arms):
        start_idx = arm_idx * arm_dim
        pose_base = data[start_idx:start_idx + 6]  # xyzrpy (6D)
        gripper = data[start_idx + 6]  # gripper (1D)
        
        # Transform pose from base to world frame
        base_in_world = np.array(arm_base_in_world_list[arm_idx], dtype=np.float32)
        pose_world = transform_pose_to_world(pose_base, base_in_world)
        
        # Update result
        result[start_idx:start_idx + 6] = pose_world
        result[start_idx + 6] = gripper  # Gripper remains unchanged
    
    return result.tolist()


def convert_poses_to_pose6d(vec: list | np.ndarray, use_row_pose6d: bool = False) -> list[float]:
    """Convert arm poses from xyzrpy to 6D pose representation.
    
    Input format: Each arm has 7 dimensions (xyzrpy 6D + gripper 1D).
                  For N arms, total dimension is 7*N.
    
    Output format: Each arm has 10 dimensions (xyz 3D + 6D rotation + gripper 1D).
                   For N arms, total dimension is 10*N.
    
    Args:
        vec: Input pose vector with xyzrpy format
        use_row_pose6d: If True, use row-major format [x, y, z, r00, r01, r10, r11, r20, r21]
                      If False (default), use column-major format [x, y, z, r00, r10, r20, r01, r11, r21]
    
    Returns:
        Output pose vector with 6D pose format
    """
    data = np.asarray(vec, dtype=np.float32)
    
    # Each arm has 7 dimensions (xyzrpy 6D + gripper 1D)
    arm_dim_input = 7
    num_arms = len(data) // arm_dim_input
    
    if len(data) % arm_dim_input != 0:
        raise ValueError(f"Data dimension {len(data)} is not a multiple of {arm_dim_input} (expected 7*N for N arms)")
    
    result_parts = []
    
    # Process each arm
    for arm_idx in range(num_arms):
        start_idx = arm_idx * arm_dim_input
        xyzrpy = data[start_idx:start_idx + 6]  # xyzrpy (6D)
        gripper = data[start_idx + 6]  # gripper (1D)
        
        # Convert xyzrpy to 6D pose (9D: xyz + 6D rotation)
        pose_6d = xyzrpy_to_pose6d(xyzrpy, use_row_pose6d=use_row_pose6d)
        
        # Combine: 6D pose (9D) + gripper (1D) = 10D per arm
        result_parts.append(np.concatenate([pose_6d, [gripper]]))
    
    # Concatenate all arms
    result = np.concatenate(result_parts).astype(np.float32)
    
    return result.tolist()


def create_empty_dataset(
    args,
    mode: Literal["video", "image"] = "video",
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
) -> LeRobotDataset:
    states = []
    actions = []
    if len(states) == 0:
        for i in range(len(args.armJointStateNames)):
            if "puppet" in args.armJointStateNames[i]:
                for j in range(args.armJointStateDims[i]):
                    states += [f'arm.jointStatePosition.{args.armJointStateNames[i]}.joint{j}']
            if "master" in args.armJointStateNames[i]:
                for j in range(args.armJointStateDims[i]):
                    actions += [f'arm.jointStatePosition.{args.armJointStateNames[i]}.joint{j}']

    if len(states) == 0:
        for i in range(len(args.armEndPoseNames)):
            if "puppet" in args.armEndPoseNames[i]:
                for j in range(args.armEndPoseDims[i]):
                    states += [f'arm.endPose.{args.armEndPoseNames[i]}.joint{j}']
            if "master" in args.armEndPoseNames[i]:
                for j in range(args.armEndPoseDims[i]):
                    actions += [f'arm.endPose.{args.armEndPoseNames[i]}.joint{j}']

    if len(states) == 0:
        for i in range(len(args.localizationPoseNames)):
            if args.convert_to_pose6d:
                # 6D pose format: xyz + 6D rotation (9D) + gripper = 10D per arm
                states += [f'localization.pose.{args.localizationPoseNames[i]}.x']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.y']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.z']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.r00']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.r01']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.r10']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.r11']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.r20']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.r21']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.gripper']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.x']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.y']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.z']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.r00']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.r01']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.r10']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.r11']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.r20']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.r21']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.gripper']
            else:
                # xyzrpy format: xyzrpy (6D) + gripper = 7D per arm
                states += [f'localization.pose.{args.localizationPoseNames[i]}.x']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.y']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.z']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.roll']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.pitch']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.yaw']
                states += [f'localization.pose.{args.localizationPoseNames[i]}.gripper']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.x']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.y']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.z']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.roll']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.pitch']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.yaw']
                actions += [f'localization.pose.{args.localizationPoseNames[i]}.gripper']
    features = {
        "observation.state": {
            "dtype": "float64",
            "shape": (len(states),),
            "names": [
                states,
            ],
        },
        "action": {
            "dtype": "float64",
            "shape": (len(actions),),
            "names": [
                actions,
            ],
        }
    }
    for camera in args.cameraColorNames:
        features[f"observation.images.{camera}"] = {
            "dtype": mode,
            "shape": (3, args.imageHeight, args.imageWidth),
            "names": [
                "channels",
                "height",
                "width",
            ],
        }
    for camera in args.cameraDepthNames:
        features[f"observation.depths.{camera}"] = {
            "dtype": "image",
            "shape": (args.imageHeight, args.imageWidth),
            "names": [
                "height",
                "width",
            ],
        }
    for force6d in args.force6dimNames:
        features[f"observation.force6d.{force6d}"] = {
            "dtype": "float64",
            # Use a 1D tuple for shape so len(ft["shape"]) is valid in LeRobot
            "shape": (6,),
            "names": [
                "force_x",
                "force_y",
                "force_z",
                "torque_x",
                "torque_y",
                "torque_z",
            ],
        }
    for arrayFloat32 in args.arrayFloat32Names:
        features[f"observation.force3d.{arrayFloat32}"] = {
            "dtype": "float64",
            "shape": (35, 20, 3),
            "names": ["height", "width", "channels"],
        }
    if args.useCameraPointCloud:
        for camera in args.cameraPointCloudNames:
            features[f"observation.pointClouds.{camera}"] = {
                "dtype": "float64",
                "shape": ((args.pointNum * 6),)
            }

    return LeRobotDataset.create(
        repo_id=args.datasetName,
        root=args.targetDir,
        fps=args.fps,
        robot_type=args.robotType,
        features=features,
        use_videos=dataset_config.use_videos,
        tolerance_s=dataset_config.tolerance_s,
        image_writer_processes=dataset_config.image_writer_processes,
        image_writer_threads=dataset_config.image_writer_threads,
        video_backend=dataset_config.video_backend,
    )


def load_episode_data(
    args,
    episode_path: Path,
):
    with h5py.File(episode_path, "r") as episode:
        try:
            states = None
            if states is None and len(args.armJointStateNames) > 0:
                states = torch.from_numpy(
                    np.concatenate(
                        [episode[f"arm/jointStatePosition/{name}"][()] for name in args.armJointStateNames if "puppet" in name], axis=1
                    )
                )
                actions = torch.from_numpy(
                    np.concatenate(
                        [episode[f"arm/jointStatePosition/{name}"][()] for name in args.armJointStateNames if "master" in name], axis=1
                    )
                )
            if states is None and len(args.armEndPoseNames) > 0:
                states = torch.from_numpy(
                    np.concatenate(
                        [episode[f"arm/endPose/{name}"][()] for name in args.armEndPoseNames if "puppet" in name], axis=1
                    )
                )
                actions = torch.from_numpy(
                    np.concatenate(
                        [episode[f"arm/endPose/{name}"][()] for name in args.armEndPoseNames if "master" in name], axis=1
                    )
                )
            if states is None and len(args.localizationPoseNames) > 0:
                # Load raw data: concatenate pose (6D) + gripper (1D) for each arm
                states_data = np.concatenate(
                    [np.concatenate((episode[f"localization/pose/{name}"][()], episode[f"gripper/encoderDistance/{name}"][()].reshape(-1, 1)), axis=1) for name in args.localizationPoseNames], axis=1
                )
                actions_data = np.concatenate(
                    [np.concatenate((episode[f"localization/pose/{name}"][()], episode[f"gripper/encoderDistance/{name}"][()].reshape(-1, 1)), axis=1) for name in args.localizationPoseNames], axis=1
                )
                
                # Apply transformations if enabled
                num_arms = len(args.localizationPoseNames)
                if args.convert_to_world_frame and num_arms > 0 and len(args.arm_base_link_in_world) >= num_arms:
                    # Convert from base frame to world frame
                    # Prepare base poses for each arm
                    arm_base_in_world_list = [
                        np.array(args.arm_base_link_in_world[i], dtype=np.float32)
                        for i in range(num_arms)
                    ]
                    
                    # Apply conversion to each row
                    states_converted = []
                    for row in states_data:
                        states_converted.append(convert_poses_to_world(row, arm_base_in_world_list))
                    states_data = np.array(states_converted, dtype=np.float32)
                    
                    actions_converted = []
                    for row in actions_data:
                        actions_converted.append(convert_poses_to_world(row, arm_base_in_world_list))
                    actions_data = np.array(actions_converted, dtype=np.float32)
                elif args.convert_to_world_frame:
                    raise ValueError(
                        f"Number of base poses ({len(args.arm_base_link_in_world)}) "
                        f"doesn't match number of arms ({num_arms}). "
                        f"Need {num_arms} base poses (each with 6 values: x, y, z, roll, pitch, yaw)."
                    )
                
                if args.convert_to_pose6d:
                    # Convert from xyzrpy to 6D pose representation
                    states_converted = []
                    for row in states_data:
                        states_converted.append(convert_poses_to_pose6d(row, use_row_pose6d=args.use_row_pose6d))
                    states_data = np.array(states_converted, dtype=np.float32)
                    
                    actions_converted = []
                    for row in actions_data:
                        actions_converted.append(convert_poses_to_pose6d(row, use_row_pose6d=args.use_row_pose6d))
                    actions_data = np.array(actions_converted, dtype=np.float32)
                
                states = torch.from_numpy(states_data)
                actions = torch.from_numpy(actions_data)
            colors = {}
            for camera in args.cameraColorNames:
                colors[camera] = []
                for i in range(episode[f'camera/color/{camera}'].shape[0]):
                    if episode[f'/camera/color/{camera}'].ndim == 1:
                        colors[camera].append(os.path.join(os.path.dirname(str(episode_path)), episode[f'camera/color/{camera}'][i].decode('utf-8')))
                        # colors[camera].append(cv2.cvtColor(cv2.imread(
                        #    os.path.join(os.path.dirname(str(episode_path)), episode[f'camera/color/{camera}'][i].decode('utf-8')),
                        #    cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB))
                    else:
                        colors[camera].append(episode[f'camera/color/{camera}'][i])
                colors[camera] = colors[camera]
            depths = {}
            for camera in args.cameraDepthNames:
                depths[camera] = []
                for i in range(episode[f'camera/depth/{camera}'].shape[0]):
                    if episode[f'/camera/depth/{camera}'].ndim == 1:
                        depth_img = cv2.imread(
                            os.path.join(os.path.dirname(str(episode_path)), episode[f'camera/depth/{camera}'][i].decode('utf-8')),
                            cv2.IMREAD_UNCHANGED)
                    else:
                        depth_img = episode[f'camera/depth/{camera}'][i]
                    depth_img = depth_img.astype(np.uint16)
                    depths[camera].append(depth_img)
            forces = {}
            for force6d in args.force6dimNames:
                forces[force6d] = []
                for i in range(episode[f'force/6dim/{force6d}'].shape[0]):
                    forces[force6d].append(episode[f'force/6dim/{force6d}'][i])
            arrayFloat32s = {}
            for arrayFloat32 in args.arrayFloat32Names:
                arrayFloat32s[arrayFloat32] = []
                for i in range(episode[f'array/float32Data/{arrayFloat32}'].shape[0]):
                    arrayFloat32s[arrayFloat32].append(episode[f'array/float32Data/{arrayFloat32}'][i].reshape(episode[f'array/float32Shape/{arrayFloat32}'][i]))
            pointclouds = {}
            if args.useCameraPointCloud:
                for camera in args.cameraPointCloudNames:
                    pointclouds[camera] = []
                    for i in range(episode[f'camera/pointCloud/{camera}'].shape[0]):
                        pointclouds[camera].append(np.load(
                            os.path.join(os.path.dirname(str(episode_path)), episode[f'camera/color/{camera}'][i].decode('utf-8'))))
            return colors, depths, pointclouds, forces, arrayFloat32s, states, actions
        except:
            return None, None, None, None, None, None, None

def populate_dataset(
    args,
    dataset: LeRobotDataset,
    hdf5_files: list[Path],
) -> LeRobotDataset:
    error_file = []
    episodes = range(len(hdf5_files))
    for ep_idx in tqdm.tqdm(episodes):
        episode_path = hdf5_files[ep_idx]
        task = 'null'
        try:
            with h5py.File(episode_path, 'r') as root:
                task = choice(root[f'instructions/full_instructions/text'][()]).decode('utf-8')
        except:
            pass
        colors, depths, pointclouds, forces, arrayFloat32s, states, actions = load_episode_data(args, episode_path)
        if colors is not None:
            num_frames = states.shape[0]

            for i in range(num_frames):
                frame = {
                    # 'task': task,
                    "observation.state": states[i],
                    "action": actions[i],
                }
                for force6d, force in forces.items():
                    frame[f"observation.force6d.{force6d}"] = force[i]
                for arrayFloat32, arrayFloat32Data in arrayFloat32s.items():
                    frame[f"observation.force3d.{arrayFloat32}"] = arrayFloat32Data[i]
                for camera, color in colors.items():
                    frame[f"observation.images.{camera}"] = color[i]
                for camera, depth in depths.items():
                    frame[f"observation.depths.{camera}"] = depth[i]
                if args.useCameraPointCloud:
                    for camera, pointcloud in pointclouds.items():
                        frame[f"observation.pointClouds.{camera}"] = pointcloud[i]
                dataset.add_frame(frame, task)
            dataset.save_episode()
        else:
            error_file.append(episode_path)
    # print("error:", error_file)
    return dataset


def process(
    args,
    push_to_hub: bool = False,
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
):
    hdf5_files = []
    if Path(args.targetDir).exists():
        shutil.rmtree(Path(args.targetDir))

    for datasetDir in args.datasetDir:
        dataset_dir = Path(datasetDir)
        if not dataset_dir.exists():
            raise ValueError(f"{dataset_dir} does not exist")
        for f in os.listdir(dataset_dir):
            if f.endswith(".hdf5"):
                hdf5_files.append(os.path.join(dataset_dir, f))
            if os.path.isdir(os.path.join(dataset_dir, f)):
                hdf5_files.extend(glob.glob(os.path.join(dataset_dir, f, "*.hdf5")))
    hdf5_files = sorted(hdf5_files)
    dataset = create_empty_dataset(
        args,
        dataset_config=dataset_config,
    )
    dataset = populate_dataset(
        args,
        dataset,
        hdf5_files
    )
    
    # Generate stats.json for backward compatibility
    from lerobot.datasets.utils import write_stats
    if dataset.meta.stats:
        write_stats(dataset.meta.stats, dataset.root)
        print(f"Stats written to {dataset.root / 'meta/stats.json'}")
    else:
        print("Warning: No stats available to write")

    # persist the sorted list into targetDir/meta/episode_list.txt
    meta_dir = Path(args.targetDir) / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    episode_list_path = meta_dir / "episode_list.txt"
    with open(episode_list_path, "w", encoding="utf-8") as f:
        for file_path in hdf5_files:
            f.write(f"{Path(file_path).stem}\n")
    print(f"Saved HDF5 list to {episode_list_path}")

    if push_to_hub:
        dataset.push_to_hub()


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir', nargs='+', required=True)
    parser.add_argument('--datasetName', action='store', type=str, help='datasetName',
                        default="data", required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--targetDir', action='store', type=str, help='targetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--robotType', action='store', type=str, help='robotType',
                        default="cobot_magic", required=False)
    parser.add_argument('--fps', action='store', type=int, help='fps',
                        default=30, required=False)
    parser.add_argument('--imageWidth', action='store', type=int, help='imageWidth',
                        default=1280, required=False)
    parser.add_argument('--imageHeight', action='store', type=int, help='imageHeight',
                        default=720, required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                        default=[], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                        default=[], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=[], required=False)
    parser.add_argument('--useCameraPointCloud', action='store', type=bool, help='useCameraPointCloud',
                        default=False, required=False)
    parser.add_argument('--pointNum', action='store', type=int, help='point_num',
                        default=5000, required=False)
    parser.add_argument('--armJointStateNames', action='store', type=str, help='armJointStateNames',
                        default=[], required=False)
    parser.add_argument('--armJointStateDims', action='store', type=int, help='armJointStateDims',
                        default=[], required=False)
    parser.add_argument('--armEndPoseNames', action='store', type=str, help='armEndPoseNames',
                        default=[], required=False)
    parser.add_argument('--armEndPoseDims', action='store', type=int, help='armEndPoseDims',
                        default=[], required=False)
    parser.add_argument('--localizationPoseNames', action='store', type=str, help='localizationPoseNames',
                        default=[], required=False)
    parser.add_argument('--gripperEncoderNames', action='store', type=str, help='gripperEncoderNames',
                        default=[], required=False)
    parser.add_argument('--imu9AxisNames', action='store', type=str, help='imu9AxisNames',
                        default=[], required=False)
    parser.add_argument('--lidarPointCloudNames', action='store', type=str, help='lidarPointCloudNames',
                        default=[], required=False)
    parser.add_argument('--robotBaseOdometryNames', action='store', type=str, help='robotBaseOdometryNames',
                        default=[], required=False)
    parser.add_argument('--robotBaseVelocityNames', action='store', type=str, help='robotBaseVelocityNames',
                        default=[], required=False)
    parser.add_argument('--liftMotorNames', action='store', type=str, help='liftMotorNames',
                        default=[], required=False)
    parser.add_argument('--convert_to_world_frame', action='store', 
                        help='Convert arm poses from base frame to world frame',
                        default=False, required=False)
    parser.add_argument('--arm_base_link_in_world', action='store', type=float, nargs='+',
                        help='arm_base_link_in_world poses: [arm0_x, arm0_y, arm0_z, arm0_roll, arm0_pitch, arm0_yaw, arm1_x, ...]. '
                             'Each arm needs 6 values (x, y, z, roll, pitch, yaw). '
                             'If not provided or incomplete, defaults will be used.',
                        default=None, required=False)
    parser.add_argument('--convert_to_pose6d', action='store',
                        help='Convert arm poses from xyzrpy to 6D pose representation. '
                             'Each arm: 7D (xyzrpy+gripper) -> 10D (xyz+6D_rotation+gripper)',
                        default=True, required=False)
    parser.add_argument('--use_row_pose6d', action='store',
                        help='Use row-major format for 6D pose: [x, y, z, r00, r01, r10, r11, r20, r21]. '
                             'Default (False) uses column-major format: [x, y, z, r00, r10, r20, r01, r11, r21]',
                        default=False, required=False)
    args = parser.parse_args()

    with open(f'../config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.cameraColorNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('color', {}).get('names', [])
        args.cameraDepthNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('depth', {}).get('names', [])
        args.cameraPointCloudNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('pointCloud', {}).get('names', [])
        args.armJointStateNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('jointState', {}).get('names', [])
        args.armEndPoseNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('endPose', {}).get('names', [])
        args.localizationPoseNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('localization', {}).get('pose', {}).get('names', [])
        args.force6dimNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('force', {}).get('6dim', {}).get('names', [])
        args.gripperEncoderNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('gripper', {}).get('encoder', {}).get('names', [])
        args.imu9AxisNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('imu', {}).get('9axis', {}).get('names', [])
        args.arrayFloat32Names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('array', {}).get('float32', {}).get('names', [])
        args.lidarPointCloudNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lidar', {}).get('pointCloud', {}).get('names', [])
        args.robotBaseOdometryNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('odometry', {}).get('names', [])
        args.robotBaseVelocityNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('velocity', {}).get('names', [])
        args.liftMotorNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lift', {}).get('motor', {}).get('names', [])
        # args.armJointStateNames = []
        # args.armEndPoseNames = []
        # args.localizationPoseNames = []
        args.armJointStateDims = [7 for _ in range(len(args.armJointStateNames))]
        args.armEndPoseDims = [7 for _ in range(len(args.armEndPoseNames))]
    
    # Parse arm_base_link_in_world based on actual number of arms
    num_arms = len(args.localizationPoseNames) if len(args.localizationPoseNames) > 0 else 0
    if args.arm_base_link_in_world is not None:
        # Each arm needs 6 values (x, y, z, roll, pitch, yaw)
        values_per_arm = 6
        total_values = len(args.arm_base_link_in_world)
        
        if total_values % values_per_arm != 0:
            raise ValueError(
                f"arm_base_link_in_world has {total_values} values, which is not a multiple of {values_per_arm}. "
                f"Each arm needs {values_per_arm} values (x, y, z, roll, pitch, yaw)."
            )
        
        num_provided_arms = total_values // values_per_arm
        
        if num_arms > 0 and num_provided_arms != num_arms:
            print(f"Warning: Number of provided base poses ({num_provided_arms}) doesn't match number of arms ({num_arms}).")
            if num_provided_arms < num_arms:
                print(f"Using default values for missing arms.")
        
        # Parse into list of 6-element arrays
        args.arm_base_link_in_world = [
            args.arm_base_link_in_world[i*values_per_arm:(i+1)*values_per_arm]
            for i in range(num_provided_arms)
        ]
        
        # If we have fewer poses than arms, pad with default values
        if num_arms > num_provided_arms:
            default_pose = [0, 0, 0, 0, 0, 0]
            args.arm_base_link_in_world.extend([default_pose] * (num_arms - num_provided_arms))
    else:
        # Use default values: all zeros for all arms
        if num_arms > 0:
            args.arm_base_link_in_world = [[0, 0, 0, 0, 0, 0] for _ in range(num_arms)]
        else:
            args.arm_base_link_in_world = []
    
    return args


def main():
    args = get_arguments()
    process(args)


if __name__ == "__main__":
    main()
