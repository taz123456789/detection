
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32.hpp> 
using namespace std;
using namespace cv;
class ColorDetectionNode : public rclcpp::Node {
public:
    ColorDetectionNode() : Node("color_detection") {
        this->declare_parameter<std::string>("target_color", "yellow"); // yellow as a default
        timer_ = this->create_wall_timer( chrono::milliseconds(30),
            bind(&ColorDetectionNode::process_frame, this)
        );
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("target_angle", 10);//publisher
        RCLCPP_INFO(this->get_logger(), "color detection started");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher_;  // publisher for the angle
    VideoCapture camera_;

    // Parameters 
    const double CAMERA_FOV_DEG = 60.0;

    void process_frame() {
        if (!camera_.isOpened()) {
            camera_.open(0);
            if (!camera_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), " camera not opening");
                return;
            }
        }

        Mat frame;
        camera_ >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "empty frame");
            return;
        }

        // HSV
        Mat hsvFrame;
        cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

        // selected color 
        string target_color;
        this->get_parameter("target_color", target_color);

        // HSV ranges
        Scalar lowerColor, upperColor;
        if (target_color == "yellow") {
            lowerColor = Scalar(20, 100, 100);
            upperColor = Scalar(40, 255, 255);
        }
        else if (target_color == "blue") {
            lowerColor = Scalar(100, 50, 50);
            upperColor = Scalar(140, 255, 255);
        }
        else if (target_color == "black") {
            lowerColor = Scalar(0, 0, 0);
            upperColor = Scalar(180, 255, 50);
        }
        else if (target_color == "white") {
            lowerColor = Scalar(0, 0, 200);
            upperColor = Scalar(180, 30, 255);
        }
        else if (target_color == "red") {
            Scalar lowerRed1(0, 50, 50), upperRed1(10, 255, 255);
            Scalar lowerRed2(160, 50, 50), upperRed2(180, 255, 255);
            Mat redMask1, redMask2;
            inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
            inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
            addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask1);
            detect_color(frame, redMask1, "Red");
            return;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "invalid color! so its yellow");
            lowerColor = Scalar(20, 100, 100);
            upperColor = Scalar(40, 255, 255);
        }

        // mask 
        Mat colorMask;
        inRange(hsvFrame, lowerColor, upperColor, colorMask);
        detect_color(frame, colorMask, target_color);
    }

    void detect_color(Mat& frame, Mat& mask, const string& color_name) {
        // contours
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // largest contour
            double maxArea = 0;
            vector<Point> largestContour;
            for (const auto& contour : contours) {
                double area = contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (!largestContour.empty()) {
                // centroid of the largest detected object
                Moments M = moments(largestContour);
                if (M.m00 != 0) {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);
                    circle(frame, Point(cx, cy), 5, Scalar(0, 255, 0), -1);

                    // angle θ relative to the center of the camera
                    int frameWidth = frame.cols;
                    double angle = ((cx - frameWidth / 2.0) / (frameWidth / 2.0)) * (CAMERA_FOV_DEG / 2.0);

                    // publishign the angle 
                    auto angle_msg = std_msgs::msg::Float32();
                    angle_msg.data = static_cast<float>(angle);
                    angle_publisher_->publish(angle_msg);

                    RCLCPP_INFO(this->get_logger(), "%s detected at angle: %.2f degrees", color_name.c_str(), angle);
                }
            }
        }

        // show frames
        imshow("Stream", frame);
        imshow(color_name + " Detection", mask);

        if (waitKey(30) == 'q') {
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectionNode>());
    rclcpp::shutdown();
    return 0;
}