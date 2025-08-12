"""
Script to convert Aloha hdf5 data to the LeRobot dataset v2.0 format.

Example usage: uv run examples/aloha_real/convert_aloha_data_to_lerobot.py --raw-dir /path/to/raw/data --repo-id <org>/<dataset-name>
"""

import dataclasses
from pathlib import Path
import shutil
from typing import Literal

import h5py
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
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


@dataclasses.dataclass(frozen=True)
class DatasetConfig:
    use_videos: bool = True
    tolerance_s: float = 0.0001
    image_writer_processes: int = 10
    image_writer_threads: int = 5
    video_backend: str | None = None


DEFAULT_DATASET_CONFIG = DatasetConfig()


def create_empty_dataset(
    args,
    mode: Literal["video", "image"] = "video",
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
) -> LeRobotDataset:
    states = []
    actions = []
    for i in range(len(args.armJointStateNames)):
        if "puppet" in args.armJointStateNames[i]:
            for j in range(args.armJointStateDims[i]):
                states += [f'arm.jointStatePosition.{args.armJointStateNames[i]}.joint{j}']
        if "master" in args.armJointStateNames[i]:
            for j in range(args.armJointStateDims[i]):
                actions += [f'arm.jointStatePosition.{args.armJointStateNames[i]}.joint{j}']

    for i in range(len(args.armEndPoseNames)):
        if "puppet" in args.armEndPoseNames[i]:
            for j in range(args.armEndPoseDims[i]):
                states += [f'arm.endPose.{args.armEndPoseNames[i]}.joint{j}']
        if "master" in args.armEndPoseNames[i]:
            for j in range(args.armEndPoseDims[i]):
                actions += [f'arm.endPose.{args.armEndPoseNames[i]}.joint{j}']

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
            "shape": (3, 480, 640),
            "names": [
                "channels",
                "height",
                "width",
            ],
        }
    # for camera in args.cameraDepthNames:
    #     features[f"observation.depths.{camera}"] = {
    #         # "dtype": mode,
    #         # "shape": (1, 480, 640),
    #         # "names": [
    #         #     "channels",
    #         #     "height",
    #         #     "width",
    #         # ],
    #         "dtype": "uint16",
    #         "shape": (480, 640)
    #     }
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
        states = torch.from_numpy(
            np.concatenate(
                [episode[f"arm/jointStatePosition/{name}"][()] for name in args.armJointStateNames if "puppet" in name] + \
                [episode[f"arm/endPose/{name}"][()] for name in args.armEndPoseNames if "puppet" in name], axis=1
            )
        )
        actions = torch.from_numpy(
            np.concatenate(
                [episode[f"arm/jointStatePosition/{name}"][()] for name in args.armJointStateNames if "master" in name] + \
                [episode[f"arm/endPose/{name}"][()] for name in args.armEndPoseNames if "master" in name], axis=1
            )
        )
        colors = {}
        for camera in args.cameraColorNames:
            colors[camera] = []
            for i in range(episode[f'camera/color/{camera}'].shape[0]):
                colors[camera].append(cv2.cvtColor(cv2.imread(
                    os.path.join(str(episode_path.resolve())[:-9], episode[f'camera/color/{camera}'][i].decode('utf-8')),
                    cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB))
            colors[camera] = colors[camera]
        depths = {}
        # for camera in args.cameraDepthNames:
        #     depths[camera] = []
        #     for i in range(episode[f'camera/depth/{camera}'].shape[0]):
        #         depths[camera].append(cv2.imread(
        #             os.path.join(str(episode_path.resolve())[:-9], episode[f'camera/depth/{camera}'][i].decode('utf-8')),
        #             cv2.IMREAD_UNCHANGED))
        pointclouds = {}
        if args.useCameraPointCloud:
            for camera in args.cameraPointCloudNames:
                pointclouds[camera] = []
                for i in range(episode[f'camera/pointCloud/{camera}'].shape[0]):
                    pointclouds[camera].append(np.load(
                        os.path.join(str(episode_path.resolve())[:-9], episode[f'camera/color/{camera}'][i].decode('utf-8'))))
    return colors, depths, pointclouds, states, actions


def populate_dataset(
    args,
    dataset: LeRobotDataset,
    hdf5_files: list[Path],
    task: str,
) -> LeRobotDataset:
    episodes = range(len(hdf5_files))

    for ep_idx in tqdm.tqdm(episodes):
        episode_path = hdf5_files[ep_idx]

        colors, depths, pointclouds, states, actions = load_episode_data(args, episode_path)
        num_frames = states.shape[0]

        for i in range(num_frames):
            frame = {
                'task': task,
                "observation.state": states[i],
                "action": actions[i],
            }
            for camera, color in colors.items():
                frame[f"observation.images.{camera}"] = color[i]
            # for camera, depth in depths.items():
            #     frame[f"observation.depths.{camera}"] = depth[i]
            if args.useCameraPointCloud:
                for camera, pointcloud in pointclouds.items():
                    frame[f"observation.pointClouds.{camera}"] = pointcloud[i]

            dataset.add_frame(frame)

        dataset.save_episode()

    return dataset


def process(
    args,
    push_to_hub: bool = False,
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
):
    dataset_dir = Path(args.datasetDir)
    if not dataset_dir.exists():
        raise ValueError("dataset_dir does not exist")
    if Path(args.targetDir).exists():
        shutil.rmtree(Path(args.targetDir))

    hdf5_files = sorted(dataset_dir.glob("**/data.hdf5"))

    dataset = create_empty_dataset(
        args,
        dataset_config=dataset_config,
    )
    dataset = populate_dataset(
        args,
        dataset,
        hdf5_files,
        task=args.instruction,
    )

    if push_to_hub:
        dataset.push_to_hub()


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--datasetName', action='store', type=str, help='datasetName',
                        default="data", required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--instruction', action='store', type=str, help='instruction',
                        default="null", required=False)
    parser.add_argument('--targetDir', action='store', type=str, help='targetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--robotType', action='store', type=str, help='robotType',
                        default="cobot_magic", required=False)
    parser.add_argument('--fps', action='store', type=int, help='fps',
                        default=30, required=False)
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
    parser.add_argument('--robotBaseVelNames', action='store', type=str, help='robotBaseVelNames',
                        default=[], required=False)
    parser.add_argument('--liftMotorNames', action='store', type=str, help='liftMotorNames',
                        default=[], required=False)
    args = parser.parse_args()

    with open(f'../config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.cameraColorNames = yaml_data['dataInfo']['camera']['color']['names']
        args.cameraDepthNames = yaml_data['dataInfo']['camera']['depth']['names']
        args.cameraPointCloudNames = yaml_data['dataInfo']['camera']['pointCloud']['names']
        args.armJointStateNames = yaml_data['dataInfo']['arm']['jointState']['names']
        args.armJointStateDims = [7 for _ in range(len(args.armJointStateNames))]
        args.armEndPoseNames = yaml_data['dataInfo']['arm']['endPose']['names']
        args.armEndPoseDims = [7 for _ in range(len(args.armEndPoseNames))]
        args.localizationPoseNames = yaml_data['dataInfo']['localization']['pose']['names']
        args.gripperEncoderNames = yaml_data['dataInfo']['gripper']['encoder']['names']
        args.imu9AxisNames = yaml_data['dataInfo']['imu']['9axis']['names']
        args.lidarPointCloudNames = yaml_data['dataInfo']['lidar']['pointCloud']['names']
        args.robotBaseVelNames = yaml_data['dataInfo']['robotBase']['vel']['names']
        args.liftMotorNames = yaml_data['dataInfo']['lift']['motor']['names']

    return args


def main():
    args = get_arguments()
    process(args)


if __name__ == "__main__":
    main()