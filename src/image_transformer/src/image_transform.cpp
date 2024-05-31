#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "cv_bridge exception: " << e.what());
        return;
    }

    cv::Mat image_output;
    // 将数据转成8-bit
    cv::normalize(cv_ptr->image, image_output, 0., 255., cv::NORM_MINMAX, CV_8U);

    cv_bridge::CvImage cv_output(cv_ptr->header, sensor_msgs::image_encodings::MONO8, image_output);
    pub.publish(cv_output.toImageMsg());

};

int main(int argc, char *argv[]) {

    string topicName = "/camera/ir/image";
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("image_transformer");
    image_transport::ImageTransport it(nh);

    auto sub = it.subscribe(topicName, 1, imageCallback);
    pub = it.advertise("/camera/ir/image_mono8", 1);

    rclcpp::spin(nh);

    rclcpp::shutdown();

    return 0;
}
