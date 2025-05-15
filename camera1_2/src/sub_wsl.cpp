#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

// 동영상 파일 저장 경로 (변경 가능)
std::string output_file = "output_video.mp4";

// VideoWriter 객체 선언 (동영상 저장용)
cv::VideoWriter writer;

// 서브스크라이버 콜백 함수
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 수신한 이미지를 OpenCV Mat 형식으로 디코딩 (원본 영상)
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    if (!frame.empty()) {
        // VideoWriter로 원본 영상을 파일에 기록
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
    // 동영상 파일을 MP4 형식으로 저장
    writer.open(output_file, cv::VideoWriter::fourcc('H', '2', '6', '4'), 30.0, cv::Size(640, 360), true);
    
    if (!writer.isOpened()) { 
        RCLCPP_ERROR(node->get_logger(), "Failed to open video file for writing!");
        rclcpp::shutdown(); 
        return -1;
    }

    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    // 서브스크라이버 생성
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
