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
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

typedef typename sensor_msgs::msg::Image Image;

class Sender : public rclcpp::Node {
    public:
        Sender(uint32_t width, uint32_t height) : Node("sender"), width_(width), height_(height) {
            publisher_ = this->create_publisher<Image>("/delay_est/q_d", QUEUE_DEPTH_);
            subscriber_ = this->create_subscription<Image>("/delay_est/q_m", QUEUE_DEPTH_, bind(&Sender::subscriptionCallback, this, _1));

            publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(PUBLISH_PERIOD_), std::bind(&Sender::publishCallback, this));

            is_collecting_data_ = true;
            data_collection_done_ = false;

            message_ = std::make_shared<Image>();
            message_->height = height_;
            message_->width = width_;
            message_->encoding = "rgb8";
            message_->is_bigendian = 0;
            message_->step = width_*3;
            message_->data = std::vector<uint8_t>(width_*height_*3, 0);
            
            RCLCPP_INFO(this->get_logger(), "Sender node initialized");
        }

        ~Sender() {
            if(log_file_.is_open()) {
                log_file_.close();
            }
        }

        bool isDone() {
            return data_collection_done_;
        }
        
    private:
        void publishCallback() {
            tx_time_ = this->now();
            message_->header.stamp = tx_time_;

            publisher_->publish(*message_);
        }

        void subscriptionCallback(const Image::SharedPtr message) {
            if(is_collecting_data_) {
                if(!data_collection_timer_) {
                    data_collection_timer_ = this->create_wall_timer(std::chrono::duration<double>(DATA_COLLECTION_PERIOD_), std::bind(&Sender::dataCollectionCallback, this));

                    RCLCPP_INFO(this->get_logger(), "Data collection started");
                }

                rx_time_ = this->now();

                samples_.push_back(rx_time_.nanoseconds());
                samples_.push_back((rx_time_ - message->header.stamp).nanoseconds());
            }
        }

        void dataCollectionCallback() {
            is_collecting_data_ = false;
            
            if(publish_timer_ && !publish_timer_->is_ready()) {
                publish_timer_->cancel();
            }
            
            if(data_collection_timer_ && !data_collection_timer_->is_ready()) {
                data_collection_timer_->cancel();
            }

            if(log_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to write data to file: file already open");
            }
            
            else {
                std::string file_name = std::to_string(width_) + "x" + std::to_string(height_) + "/" + "receiver_to_sender.csv";
                log_file_.open(std::filesystem::path(PATH_) / file_name, std::ios::out);

                log_file_ << "timestamp,delay\n";
                for(size_t i = 0; i < samples_.size(); i += 2) {
                    log_file_ << samples_.at(i) << "," << samples_.at(i + 1) << "\n";
                }

                log_file_.close();
            }

            RCLCPP_INFO(this->get_logger(), "Data collection finalized");

            data_collection_done_ = true;
        }

        static constexpr size_t QUEUE_DEPTH_ = 10;

        static constexpr double PUBLISH_FREQUENCY_ = 15.0;
        static constexpr double PUBLISH_PERIOD_ = 1.0/PUBLISH_FREQUENCY_;
        
        static constexpr double DATA_COLLECTION_PERIOD_ = 1200.0;

        static constexpr std::string_view PATH_ = "/home/ros2user/data/payload/";

        uint32_t width_;
        uint32_t height_;

        rclcpp::Publisher<Image>::SharedPtr publisher_;
        rclcpp::Subscription<Image>::SharedPtr subscriber_;

        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr data_collection_timer_;

        bool is_collecting_data_;
        bool data_collection_done_;

        Image::SharedPtr message_;

        rclcpp::Time tx_time_;
        rclcpp::Time rx_time_;

        std::vector<int64_t> samples_;

        std::ofstream log_file_;
};

int main(int argc, char* argv[]) {
    uint32_t w = 40;
    uint32_t h = 35;

    for(size_t r = 0; r < 4; r++) {
        rclcpp::init(argc, argv);
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing %dx%d", w, h);

        auto sender_node = std::make_shared<Sender>(w, h);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(sender_node);

        std::thread spin_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));
        
        bool done = false;
        while(!done) {
            done = sender_node->isDone();
            std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
        }

        RCLCPP_INFO(rclcpp::get_logger("main"), "Terminating %dx%d", w, h);

        w *= 2;
        h *= 2;

        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
    }

    return 0;
}