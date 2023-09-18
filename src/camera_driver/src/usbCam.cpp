#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
using namespace std::chrono_literals;
using namespace cv;

class usbCamNode : public rclcpp::Node
{
public:
    usbCamNode(int port) : Node("usbCamNode"),port_(port)
    {
        RCLCPP_INFO(this->get_logger(), "usbCamNode has been started.");
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("usbCam", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&usbCamNode::timerCallback, this));
        cap_.open(port);
    }

private:
    void timerCallback()
    {
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera.");
            return;
        }
        Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get frame.");
            return;
        }
        imshow("usbCam", frame);
        if(waitKey(1) == 27)
        {
            RCLCPP_INFO(this->get_logger(), "ESC key is pressed by user.");
            rclcpp::shutdown();
        }
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }
    VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    int port_;
};

int main(int argc, char **argv)
{
    int port = 0;
    rclcpp::init(argc, argv);
    if(argc == 2)
    {
        if(strcmp(argv[1], "-h") == 0)
        {
            std::cout << "Usage: ros2 run camera_driver usbCam [video port]" << std::endl;
            return 0;
        }
        else
        {
            if(atoi(argv[1]) > 0 && atoi(argv[1]) < 10)
            {
                port = atoi(argv[1]);
            }
            else
            {
                std::cout << "ERROR INPUT" << std::endl;
                return 0;
            }
        }
        
    }

    auto node = std::make_shared<usbCamNode>(port);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}