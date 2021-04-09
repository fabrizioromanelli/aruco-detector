#include <memory>
#include <chrono>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/ArucoCodeScanner.h>
#include "rmw/qos_profiles.h"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

using std::placeholders::_1;
using namespace std::chrono_literals;

// Detector states
enum eDetectorState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    LOST=3
};

class ADetectorNode : public rclcpp::Node
{
  public:
    ADetectorNode(std::string _cameraTopic, ORB_SLAM2::ArucoCodeScanner *_aScanner) : Node("arucoDetector"), cameraTopic(_cameraTopic), aScanner(_aScanner)
    {
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

      // Create publishers with 200ms period
      img_publisher_   = this->create_publisher<sensor_msgs::msg::Image>("arucoDetector_image", qos);
          img_timer_   = this->create_wall_timer(200ms, std::bind(&ADetectorNode::timer_img_callback, this));
      state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("arucoDetector_state", qos);
          state_timer_ = this->create_wall_timer(200ms, std::bind(&ADetectorNode::timer_state_callback, this));

      // Create subscriber
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(cameraTopic, qos, std::bind(&ADetectorNode::GrabRGB, this, _1));

      arucoDetectorState = eDetectorState::NO_IMAGES_YET;
    }

    void setState(signed int);
    void setImage(cv::Mat, std_msgs::msg::Header);

  private:
    void timer_img_callback();
    void timer_state_callback();
    void GrabRGB(const sensor_msgs::msg::Image::SharedPtr);

    rclcpp::TimerBase::SharedPtr img_timer_, state_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    signed int arucoDetectorState = eDetectorState::SYSTEM_NOT_READY;
    std::mutex imgMtx;
    std::mutex stateMtx;
    cv::Mat detectedImage;
    std_msgs::msg::Header imgHeader;
    std::string cameraTopic;
    ORB_SLAM2::ArucoCodeScanner *aScanner;
};

void ADetectorNode::setState(signed int _state)
{
  stateMtx.lock();
  arucoDetectorState = _state;
  stateMtx.unlock();
}

void ADetectorNode::setImage(cv::Mat _inputImg, std_msgs::msg::Header _imgHeader)
{
  imgMtx.lock();
  detectedImage = _inputImg;
  imgHeader     = _imgHeader;
  imgMtx.unlock();
}

void ADetectorNode::timer_img_callback()
{
  sensor_msgs::msg::Image message = sensor_msgs::msg::Image();
  imgMtx.lock();
  stateMtx.lock();
  if (arucoDetectorState == eDetectorState::OK)
  {
    cv_bridge::CvImage message;
    message.header   = imgHeader;
    message.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    message.image    = detectedImage;

    img_publisher_->publish(*message.toImageMsg());
  }
  imgMtx.unlock();
  stateMtx.unlock();
}

void ADetectorNode::timer_state_callback()
{
  auto message = std_msgs::msg::Int32();
  stateMtx.lock();
  message.data = arucoDetectorState;
  stateMtx.unlock();
  state_publisher_->publish(message);
}

void ADetectorNode::GrabRGB(const sensor_msgs::msg::Image::SharedPtr msgRGB)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
  std_msgs::msg::Header imgHeader = cv_ptrRGB->header;
  if(aScanner->Detect(cv_ptrRGB->image))
  {
    cv::Mat detectedImage;
    aScanner->getImage(detectedImage);
    this->setImage(detectedImage, imgHeader);
    this->setState(eDetectorState::OK);
  }
  else
  {
    this->setState(eDetectorState::NO_IMAGES_YET);
  }
}

//
// Main
//
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string cameraTopic = "/camera/color/image_raw";

  ORB_SLAM2::ArucoCodeScanner *arucoCodeScanner = new ORB_SLAM2::ArucoCodeScanner();
  auto nodePtr = std::make_shared<ADetectorNode>(cameraTopic, arucoCodeScanner);

  rclcpp::spin(nodePtr);
  rclcpp::shutdown();
  return 0;
}
