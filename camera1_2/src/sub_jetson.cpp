#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.156 port=9004 sync=false";

cv::VideoWriter writer;

// 서브스크라이버 콜백 함수 수정
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 수신한 이미지를 OpenCV Mat 형식으로 디코딩 (원본 영상)
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    // 원본 영상을 그대로 처리
    if (!frame.empty()) {
        // VideoWriter로 프레임 기록 (원본 영상 그대로)
        writer << frame;

        // 수신한 영상에 대한 정보 로그 출력
        RCLCPP_INFO(node->get_logger(), "Received Image: %d x %d", frame.rows, frame.cols);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to decode image!");
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");

    // 비디오 파일로 출력하기 위해 VideoWriter 객체를 초기화
    writer.open(dst, 0, (double)30, cv::Size(640, 360), true);  // true: 컬러 영상 (BGR 형식)
    if (!writer.isOpened()) { 
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!");
        rclcpp::shutdown(); 
        return -1;
    }

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    // 서브스크라이버 생성
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
