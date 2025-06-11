// #include <mcap/reader.hpp>
// #include <rclcpp/serialization.hpp>
// #include <rclcpp/serialized_message.hpp>
// // #include <rosbag2_storage_mcap/mcap_storage.cpp>

// int main(int argc, char** argv) {
//   // 1. 打开 MCAP 文件
//   mcap::McapReader reader;
//   auto status = reader.open("test.mcap");
//   if (!status.ok()) {
//     std::cerr << "Failed to open MCAP file: " << status.message << std::endl;
//     return 1;
//   }

//   // 2. 读取消息
//   auto onMessage = [&](const mcap::MessageView& msgView) {
//     // 解析 ROS2 消息类型
//     std::string topic_type = msgView.channel->messageEncoding;
    
//     // 反序列化消息（以 sensor_msgs::msg::Image 为例）
//     if (topic_type == "sensor_msgs/msg/Image") {
//       rclcpp::SerializedMessage serialized_msg(msgView.dataSize);
//       memcpy(serialized_msg.get_rcl_serialized_message().buffer, 
//              msgView.data, msgView.dataSize);
      
//       rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
//       sensor_msgs::msg::Image msg;
//       serializer.deserialize_message(&serialized_msg, &msg);
      
//       std::cout << "Received Image: " << msg.header.frame_id 
//                 << ", size=" << msg.data.size() << " bytes" << std::endl;
//     }
//     return mcap::ReadMessage::Continue;
//   };

//   // 3. 遍历消息
//   reader.readMessages(onMessage);
//   reader.close();
//   return 0;
// }


#include <mcap/reader.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <mcap_file>" << std::endl;
        return 1;
    }

    const std::string filename = argv[1];

    mcap::McapReader reader;
    auto status = reader.open(filename);
    if (!status.ok()) {
        std::cerr << "Failed to open MCAP file: " << status.message << std::endl;
        return 1;
    }

    // 读取文件的统计信息
    // const auto& statistics = reader.statistics();
    // std::cout << "MCAP file summary:" << std::endl;
    // std::cout << "  Channels: " << statistics.channelCount << std::endl;
    // std::cout << "  Schemas: " << statistics.schemaCount << std::endl;
    // std::cout << "  Messages: " << statistics.messageCount << std::endl;

    // 遍历所有 Channel
    std::cout << "\nChannels:" << std::endl;
    for (const auto& [channelId, channelPtr] : reader.channels()) {
        const auto& channel = *channelPtr;
        std::cout << "  Channel ID: " << channelId << std::endl;
        std::cout << "    Topic: " << channel.topic << std::endl;
        std::cout << "    Schema ID: " << channel.schemaId << std::endl;
        std::cout << "    Encoding: " << channel.messageEncoding << std::endl;
    }

    // 遍历所有 Schema
    std::cout << "\nSchemas:" << std::endl;
    for (const auto& [schemaId, schemaPtr] : reader.schemas()) {
        const auto& schema = *schemaPtr;
        std::cout << "  Schema ID: " << schemaId << std::endl;
        std::cout << "    Name: " << schema.name << std::endl;
        std::cout << "    Encoding: " << schema.encoding << std::endl;
    }

    // 遍历所有消息
    std::cout << "\nMessages:" << std::endl;
    for (const auto& messageView : reader.readMessages()) {
        std::cout << "  Channel ID: " << messageView.channel->id << std::endl;
        std::cout << "    Sequence: " << messageView.message.sequence << std::endl;
        std::cout << "    Timestamp: " << messageView.message.logTime << std::endl;
        std::cout << "    Data size: " << messageView.message.dataSize << std::endl;

        // 如果需要解析消息内容（例如 ROS 2 消息）
        // const uint8_t* data = messageView.message.data;
        // size_t size = messageView.message.dataSize;
        // 使用 ROS 2 的消息类型反序列化数据
    }

    reader.close();
    return 0;
}
