#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

typedef typename sensor_msgs::msg::JointState Joint;

class Receiver : public rclcpp::Node {
    public:
        Receiver() : Node("receiver") {
            publisher_ = this->create_publisher<Joint>("/delay_est/q_m", QUEUE_DEPTH_);
            subscriber_ = this->create_subscription<Joint>("/delay_est/q_d", QUEUE_DEPTH_, std::bind(&Receiver::subscriptionCallback, this, _1));

            is_collecting_data_ = true;

            RCLCPP_INFO(this->get_logger(), "Receiver node initialized");
        }

        ~Receiver() {
            if(log_file_.is_open()) {
                log_file_.close();
            }
        }

    private:
        void subscriptionCallback(const Joint::SharedPtr message) {
            if(is_collecting_data_) {
                if(!data_collection_timer_) {
                    data_collection_timer_ = this->create_wall_timer(std::chrono::duration<double>(DATA_COLLECTION_PERIOD_), std::bind(&Receiver::dataCollectionCallback, this));
                    
                    RCLCPP_INFO(this->get_logger(), "Data collection started");
                }

                time_ = this->now();

                samples_.push_back(time_.nanoseconds());
                samples_.push_back((time_ - message->header.stamp).nanoseconds());

                message->header.stamp = time_;

                publisher_->publish(*message);
            }
        }

        void dataCollectionCallback() {
            is_collecting_data_ = false;

            if(data_collection_timer_ && !data_collection_timer_->is_ready()) {
                data_collection_timer_->cancel();
            }

            if(log_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to write data to file: file already open");
            }

            else {
                std::string file_name = "sender_to_receiver.csv";
                log_file_.open(std::filesystem::path(PATH_) / file_name, std::ios::out);

                log_file_ << "timestamp,delay\n";
                for(size_t i = 0; i < samples_.size(); i += 2) {
                    log_file_ << samples_.at(i) << "," << samples_.at(i + 1) << "\n";
                }

                log_file_.close();
            }

            RCLCPP_INFO(this->get_logger(), "Data collection finalized");

            rclcpp::shutdown();
        }

        static constexpr size_t QUEUE_DEPTH_ = 10;

        static constexpr double DATA_COLLECTION_PERIOD_ = 120.0;

        static constexpr std::string_view PATH_ = "/home/ros2user/data/ping-pong/";

        rclcpp::Publisher<Joint>::SharedPtr publisher_;
        rclcpp::Subscription<Joint>::SharedPtr subscriber_;

        rclcpp::TimerBase::SharedPtr data_collection_timer_;

        bool is_collecting_data_;

        rclcpp::Time time_;

        std::vector<int64_t> samples_;

        std::ofstream log_file_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    Receiver::SharedPtr receiver_node = std::make_shared<Receiver>();

    rclcpp::spin(receiver_node);

    rclcpp::shutdown();
    return 0;
}