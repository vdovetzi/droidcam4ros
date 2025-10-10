#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/videoio.hpp>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int32_t find_droidcam_device() {
    for (int i = 0; i < 99; ++i) {
        std::string device_path = "/dev/video" + std::to_string(i);
        int fd = open(device_path.c_str(), O_RDONLY);

        if (fd == -1) {
            continue;
        }

        v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 && std::string(reinterpret_cast<const char*>(cap.driver)) == "Droidcam") {
            close(fd);
            return i;
        }
        close(fd);
    }

    return -1;
}


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("droidcam_publisher");

    node->declare_parameter<int32_t>("device", find_droidcam_device());
    node->declare_parameter<std::string>("output_topic");
    node->declare_parameter<std::string>("frame_id");

    int32_t device = node->get_parameter("device").as_int();
    std::string output_topic = node->get_parameter("output_topic").as_string();
    std::string frame_id = node->get_parameter("frame_id").as_string();

    cv::VideoCapture cap;

    cap.open(device);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot open droidcam device: %d", device);
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    cv::Mat frame;
    std::unique_ptr<image_transport::ImageTransport> it = std::make_unique<image_transport::ImageTransport>(node);
    std::unique_ptr<image_transport::Publisher> publisher = std::make_unique<image_transport::Publisher>(it->advertise(output_topic, 10));

    while (rclcpp::ok()){
        cap >> frame;
        
        if (frame.empty()) {
            RCLCPP_WARN(node->get_logger(), "Empty frame captured");
            continue;
        }

        auto now = node->now();
        
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = now;
        msg->header.frame_id = frame_id;;
        
        publisher->publish(msg);
    }

    return EXIT_SUCCESS;
}