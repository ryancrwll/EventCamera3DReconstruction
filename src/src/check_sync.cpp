#include <iostream>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

using namespace std;

class TimeAnalyzer : public event_camera_codecs::EventProcessor {
public:
    bool is_master;

    uint64_t first_hw_master = 0, latest_hw_master = 0;
    uint64_t first_hw_slave = 0, latest_hw_slave = 0;

    uint64_t total_master_events = 0;
    uint64_t total_slave_events = 0;

    inline void eventCD(uint64_t sensor_time, uint16_t, uint16_t, uint8_t) override {
        if (is_master) {
            if (first_hw_master == 0) first_hw_master = sensor_time;
            latest_hw_master = sensor_time;
            total_master_events++;
        } else {
            if (first_hw_slave == 0) first_hw_slave = sensor_time;
            latest_hw_slave = sensor_time;
            total_slave_events++;
        }
    }

    inline bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return true; }
    inline void finished() override {}
    inline void rawData(const char *, size_t) override {}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    string mcap_file = "/home/ryan/Documents/MIRS/Thesis/3Dreconstruction/laser2/laser2_0.mcap";
    string master_topic = "/event_cam_0/events";
    string slave_topic = "/event_cam_1/events";

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = mcap_file;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader.open(storage_options, converter_options);

    event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, TimeAnalyzer> decoderFactory;
    rclcpp::Serialization<event_camera_msgs::msg::EventPacket> serializer;

    TimeAnalyzer analyzer;

    uint64_t first_ros_master = 0, latest_ros_master = 0;
    uint64_t first_ros_slave = 0, latest_ros_slave = 0;

    cout << "Scanning ROS Bag for Time Synchronization Data..." << endl;

    while (reader.has_next() && rclcpp::ok()) {
        auto msg = reader.read_next();
        if (msg->topic_name != master_topic && msg->topic_name != slave_topic) continue;

        analyzer.is_master = (msg->topic_name == master_topic);

        if (analyzer.is_master) {
            if (first_ros_master == 0) first_ros_master = msg->recv_timestamp;
            latest_ros_master = msg->recv_timestamp;
        } else {
            if (first_ros_slave == 0) first_ros_slave = msg->recv_timestamp;
            latest_ros_slave = msg->recv_timestamp;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_msg = std::make_shared<event_camera_msgs::msg::EventPacket>();
        serializer.deserialize_message(&serialized_msg, ros_msg.get());

        auto decoder = decoderFactory.getInstance(*ros_msg);
        if (decoder) decoder->decode(*ros_msg, &analyzer);
    }

    cout << "\n==================================================" << endl;
    cout << "              TIME SYNCHRONIZATION REPORT         " << endl;
    cout << "==================================================" << endl;

    cout << "\n--- EVENT COUNTS ---" << endl;
    cout << "Master Events: " << analyzer.total_master_events << endl;
    cout << "Slave Events:  " << analyzer.total_slave_events << endl;

    cout << "\n--- ROS NETWORK TIME (Host Computer Clock) ---" << endl;
    double ros_start_diff = (double)(first_ros_master > first_ros_slave ? first_ros_master - first_ros_slave : first_ros_slave - first_ros_master) / 1e9;
    cout << "Master First Packet: " << first_ros_master << " ns" << endl;
    cout << "Slave First Packet:  " << first_ros_slave << " ns" << endl;
    cout << "ROS Start Offset:    " << ros_start_diff << " seconds" << endl;

    cout << "\n--- HARDWARE EMBEDDED TIME (Camera Internal Clock) ---" << endl;
    double hw_start_diff = (double)(analyzer.first_hw_master > analyzer.first_hw_slave ? analyzer.first_hw_master - analyzer.first_hw_slave : analyzer.first_hw_slave - analyzer.first_hw_master) / 1e9;
    cout << "Master First Event:  " << analyzer.first_hw_master << " ns" << endl;
    cout << "Slave First Event:   " << analyzer.first_hw_slave << " ns" << endl;
    cout << "Hardware Offset:     " << hw_start_diff << " seconds" << endl;

    if (hw_start_diff > 0.05) {
        cout << "\n[WARNING] Massive Hardware Time Offset Detected!" << endl;
        cout << "The cameras are desynchronized by " << hw_start_diff << " seconds." << endl;
        cout << "If one camera is reading time = 0s and the other is reading time = " << hw_start_diff << "s," << endl;
        cout << "the 3D triangulation math will look for overlapping laser flashes that don't exist." << endl;
    } else {
        cout << "\n[SUCCESS] Hardware clocks are tightly synchronized!" << endl;
    }

    cout << "==================================================\n" << endl;

    rclcpp::shutdown();
    return 0;
}