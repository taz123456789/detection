#define M_PI 3.14159265358979323846
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

class ColorDetectionNode : public rclcpp::Node {
public:
    ColorDetectionNode() : Node("color_detection_node"), camera_(0) {
        this->declare_parameter<std::string>("target_color", "yellow");

        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ColorDetectionNode::process_frame, this)
        );

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detected_object", 10);
        RCLCPP_INFO(this->get_logger(), "Node started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    cv::VideoCapture camera_;
    const double CAMERA_FOV_DEG = 60.0;
    const double MIN_OBJECT_SIZE = 0.03;  // Smallest detectable object
    const double MAX_OBJECT_SIZE = 0.1;   // Largest object before stopping

    void process_frame() {
        cv::Mat frame;
        camera_ >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty frame");
            return;
        }

        cv::Mat hsvFrame;
        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

        std::string target_color;
        this->get_parameter("target_color", target_color);

        cv::Scalar lowerColor, upperColor;
        if (target_color == "yellow") {
            lowerColor = cv::Scalar(20, 100, 100);
            upperColor = cv::Scalar(40, 255, 255);
        }
        else if (target_color == "blue") {
            lowerColor = cv::Scalar(100, 50, 50);
            upperColor = cv::Scalar(140, 255, 255);
        }
        else if (target_color == "black") {
            lowerColor = cv::Scalar(0, 0, 0);
            upperColor = cv::Scalar(180, 255, 50);
        }
        else if (target_color == "white") {
            lowerColor = cv::Scalar(0, 0, 200);
            upperColor = cv::Scalar(180, 30, 255);
        }
        else if (target_color == "red") {
            cv::Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
            cv::Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
            cv::Mat redMask1, redMask2, colorMask;
            cv::inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
            cv::inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
            cv::bitwise_or(redMask1, redMask2, colorMask);
            detect_color(frame, colorMask, "Red");
            return;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid color! Defaulting to yellow.");
            lowerColor = cv::Scalar(20, 100, 100);
            upperColor = cv::Scalar(40, 255, 255);
        }

        cv::Mat colorMask;
        cv::inRange(hsvFrame, lowerColor, upperColor, colorMask);
        detect_color(frame, colorMask, target_color);
    }

    void detect_color(cv::Mat& frame, cv::Mat& mask, const std::string& color_name) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double maxArea = 0;
            std::vector<cv::Point> largestContour;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (!largestContour.empty()) {
                cv::Moments M = cv::moments(largestContour);
                if (M.m00 != 0) {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);

                    cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);

                    double object_size = cv::contourArea(largestContour) / (frame.cols * frame.rows);
                    int frameWidth = frame.cols;
                    double angle = ((cx - frameWidth / 2.0) / (frameWidth / 2.0)) * (CAMERA_FOV_DEG / 2.0);

                    control_robot(angle, object_size);
                    visualize_in_rviz(angle);
                }
            }
        }

        cv::imshow("Camera Stream", frame);
        cv::imshow(color_name + " Detection", mask);

        if (cv::waitKey(1) == 'q') {
            rclcpp::shutdown();
        }
    }

    void control_robot(double angle_deg, double object_size) {
        geometry_msgs::msg::Twist cmd_vel;
        double angular_gain = 0.5;
        cmd_vel.angular.z = -angular_gain * (angle_deg * M_PI / 180.0);

        if (object_size > MIN_OBJECT_SIZE) {
            cmd_vel.linear.x = std::max(0.1, 0.5 * (1.0 - object_size / MAX_OBJECT_SIZE));
        }
        else {
            cmd_vel.linear.x = 0.0;  // Stop if object is too small
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void visualize_in_rviz(double angle_deg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 2.0;
        marker.pose.position.y = -2.0 * tan(angle_deg * M_PI / 180.0);
        marker.pose.position.z = 0.5;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
