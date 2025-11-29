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

class Sender : public rclcpp::Node {
    public:
        Sender() : Node("sender") {
            publisher_ = this->create_publisher<Joint>("/delay_est/q_d", QUEUE_DEPTH_);
            subscriber_ = this->create_subscription<Joint>("/delay_est/q_m", QUEUE_DEPTH_, bind(&Sender::subscriptionCallback, this, _1));

            connection_check_timer_ = this->create_wall_timer(std::chrono::duration<double>(CONNECTION_CHECK_PERIOD_), std::bind(&Sender::connectionCheckCallback, this));

            is_collecting_data_ = true;

            message_ = std::make_shared<Joint>();
            message_->name = {"q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10"};
            message_->position = std::vector<double>(10, 0.0);
            message_->velocity = std::vector<double>(10, 0.0);
            message_->effort = std::vector<double>(10, 0.0);

            RCLCPP_INFO(this->get_logger(), "Sender node initialized");
        }

        ~Sender() {
            if(owd_log_file_.is_open()) {
                owd_log_file_.close();
            }

            if(rtt_log_file_.is_open()) {
                rtt_log_file_.close();
            }
        }

    private:
        void connectionCheckCallback() {
            tx_time_ = this->now();
            message_->header.stamp = tx_time_;

            publisher_->publish(*message_);
        }

        void subscriptionCallback(const Joint::SharedPtr message) {
            if(is_collecting_data_) {
                if(connection_check_timer_&& !connection_check_timer_->is_ready()) {
                    connection_check_timer_->cancel();
                }

                if(!data_collection_timer_) {
                    data_collection_timer_ = this->create_wall_timer(std::chrono::duration<double>(DATA_COLLECTION_PERIOD_), std::bind(&Sender::dataCollectionCallback, this));

                    RCLCPP_INFO(this->get_logger(), "Data collection started");
                }

                rx_time_ = this->now();

                owd_samples_.push_back(rx_time_.nanoseconds());
                owd_samples_.push_back((rx_time_ - message->header.stamp).nanoseconds());

                rtt_samples_.push_back(rx_time_.nanoseconds());
                rtt_samples_.push_back((rx_time_ - tx_time_).nanoseconds());

                tx_time_ = rx_time_;
                message->header.stamp = tx_time_;

                publisher_->publish(*message);
            }
        }

        void dataCollectionCallback() {
            is_collecting_data_ = false;

            if(data_collection_timer_ && !data_collection_timer_->is_ready()) {
                data_collection_timer_->cancel();
            }

            if(owd_log_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to write data to file: file already open");
            }
            
            else {
                std::string file_name = "receiver_to_sender.csv";
                owd_log_file_.open(std::filesystem::path(PATH_) / file_name, std::ios::out);

                owd_log_file_ << "timestamp,delay\n";
                for(size_t i = 0; i < owd_samples_.size(); i += 2) {
                    owd_log_file_ << owd_samples_.at(i) << "," << owd_samples_.at(i + 1) << "\n";
                }

                owd_log_file_.close();
            }

            if(rtt_log_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to write data to file: file already open");
            }
            
            else {
                std::string file_name = "round_trip.csv";
                rtt_log_file_.open(std::filesystem::path(PATH_) / file_name, std::ios::out);

                rtt_log_file_ << "timestamp,delay\n";
                for(size_t i = 0; i < rtt_samples_.size(); i += 2) {
                    rtt_log_file_ << rtt_samples_.at(i) << "," << rtt_samples_.at(i + 1) << "\n";
                }

                rtt_log_file_.close();
            }

            RCLCPP_INFO(this->get_logger(), "Data collection finalized");

            rclcpp::shutdown();
        }

        static constexpr size_t QUEUE_DEPTH_ = 10;

        static constexpr double CONNECTION_CHECK_FREQUENCY_ = 150.0;
        static constexpr double CONNECTION_CHECK_PERIOD_ = 1.0/CONNECTION_CHECK_FREQUENCY_;

        static constexpr double DATA_COLLECTION_PERIOD_ = 120.0;

        static constexpr std::string_view PATH_ = "/home/ros2user/data/ping-pong/";

        rclcpp::Publisher<Joint>::SharedPtr publisher_;
        rclcpp::Subscription<Joint>::SharedPtr subscriber_;

        rclcpp::TimerBase::SharedPtr connection_check_timer_;
        rclcpp::TimerBase::SharedPtr data_collection_timer_;

        bool is_collecting_data_;

        Joint::SharedPtr message_;

        rclcpp::Time tx_time_;
        rclcpp::Time rx_time_;

        std::vector<std::int64_t> rtt_samples_;
        std::vector<std::int64_t> owd_samples_;

        std::ofstream rtt_log_file_;
        std::ofstream owd_log_file_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    Sender::SharedPtr sender_node = std::make_shared<Sender>();

    rclcpp::spin(sender_node);

    rclcpp::shutdown();
    return 0;
}