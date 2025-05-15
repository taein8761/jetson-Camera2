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
    udpsink host = 203.234.58.156 port = 9004 sync=false";

cv::VideoWriter writer;

// 서브스크라이버 콜백 함수 수정
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 수신한 이미지를 OpenCV Mat 형식으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    
    // 컬러 이미지를 그레이 이미지로 변환
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 그레이 이미지를 이진 이미지로 변환
    cv::Mat binary;
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY); // 이진화 (임계값 128)

    // 이진 이미지를 UDP로 전송하기 전에 cv::imencode로 압축할 수도 있음
    writer << binary;  // 이진 영상을 writer에 기록

    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d, %d", msg->format.c_str(), binary.rows, binary.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");

    // 비디오 파일로 출력하기 위해 VideoWriter 객체를 초기화
    writer.open(dst, 0, (double)30, cv::Size(640, 360), false);  // false: 이진 영상은 컬러가 아니므로 false
    if(!writer.isOpened()) { 
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
