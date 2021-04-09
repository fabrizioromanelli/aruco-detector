#include <memory>
#include <chrono>
#include <mutex>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ArucoCodeScanner.h>
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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
    ADetectorNode() : Node("aruco-detector")
    {
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

      // Create publishers with 200ms period
      img_publisher_   = this->create_publisher<sensor_msgs::msg::Image>("aruco-detector_image", qos);
          img_timer_   = this->create_wall_timer(200ms, std::bind(&ADetectorNode::timer_img_callback, this));
      state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("aruco-detector_state", qos);
          state_timer_ = this->create_wall_timer(200ms, std::bind(&ADetectorNode::timer_state_callback, this));

      arucoTrackerState = eDetectorState::NO_IMAGES_YET;
    }

    void setState(signed int);
    void setImage(cv::Mat);

  private:
    void timer_img_callback();
    void timer_state_callback();

    rclcpp::TimerBase::SharedPtr img_timer_, state_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    signed int arucoTrackerState = eDetectorState::SYSTEM_NOT_READY;
    std::mutex imgMtx;
    std::mutex stateMtx;
    cv::Mat detectedImage;
};

void ADetectorNode::setState(signed int _state)
{
  stateMtx.lock();
  arucoTrackerState = _state;
  stateMtx.unlock();
}

void ADetectorNode::setImage(cv::Mat _inputImg)
{
  imgMtx.lock();
  detectedImage = _inputImg;
  imgMtx.unlock();
}

void ADetectorNode::timer_img_callback()
{
  sensor_msgs::msg::Image message = sensor_msgs::msg::Image();
  imgMtx.lock();
  if (orbslam2Pose.empty())
  {
    orbslam2Pose = cv::Mat::eye(4,4,CV_32F);
  }
  message.position.x = orbslam2Pose.at<float>(0,3);
  message.position.y = orbslam2Pose.at<float>(1,3);
  message.position.z = orbslam2Pose.at<float>(2,3);
  Eigen::Matrix3f orMat;
  orMat(0,0) = orbslam2Pose.at<float>(0,0);
  orMat(0,1) = orbslam2Pose.at<float>(0,1);
  orMat(0,2) = orbslam2Pose.at<float>(0,2);
  orMat(1,0) = orbslam2Pose.at<float>(1,0);
  orMat(1,1) = orbslam2Pose.at<float>(1,1);
  orMat(1,2) = orbslam2Pose.at<float>(1,2);
  orMat(2,0) = orbslam2Pose.at<float>(2,0);
  orMat(2,1) = orbslam2Pose.at<float>(2,1);
  orMat(2,2) = orbslam2Pose.at<float>(2,2);
  imgMtx.unlock();
  Eigen::Quaternionf q(orMat);
  message.orientation.x = q.x();
  message.orientation.y = q.y();
  message.orientation.z = q.z();
  message.orientation.w = q.w();
  img_publisher_->publish(message);
}

void ADetectorNode::timer_state_callback()
{
  auto message = std_msgs::msg::Int32();
  stateMtx.lock();
  message.data = arucoTrackerState;
  stateMtx.unlock();
  state_publisher_->publish(message);
}

class ImageGrabber
{
  public:
    ImageGrabber(std::shared_ptr<ADetectorNode> pADetectorNode, ORB_SLAM2::ArucoCodeScanner *_aScanner) : mpADetectorNode(pADetectorNode), aScanner(_aScanner){}

    void GrabRGB(const sensor_msgs::msg::Image::SharedPtr& msgRGB);

  private:
    std::shared_ptr<ADetectorNode> mpADetectorNode;
    ORB_SLAM2::ArucoCodeScanner *aScanner;
};

void ImageGrabber::GrabRGB(const sensor_msgs::msg::Image::SharedPtr& msgRGB)
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

  rclcpp::Time Ts = cv_ptrRGB->header.stamp;
  if(aScanner->Detect(cv_ptrRGB->image))
  {
    cv::Mat detectedImage;
    aScanner->getImage(detectedImage);
    mpADetectorNode->setImage(detectedImage);
    mpADetectorNode->setState(eDetectorState::OK);
  }
  else
  {
    mpADetectorNode->setState(eDetectorState::NO_IMAGES_YET);
  }
}

//
// Main
//
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  ORB_SLAM2::ArucoCodeScanner *arucoCodeScanner = new ORB_SLAM2::ArucoCodeScanner();
  auto nodePtr = std::make_shared<ADetectorNode>();

  ImageGrabber igb(nodePtr, arucoCodeScanner);
  std::string s = "/camera/color/image_raw";

  // message_filters::Subscriber<sensor_msgs::msg::Image> stream1_sub(nodePtr.get(), s1, rmw_qos_profile_sensor_data);
  // message_filters::Subscriber<sensor_msgs::msg::Image> stream2_sub(nodePtr.get(), s2, rmw_qos_profile_sensor_data);
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
  // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), stream1_sub, stream2_sub);

  // if (sensorType == ORB_SLAM2::System::RGBD)
  //   sync.registerCallback(&ImageGrabber::GrabRGBD, &igb);
  // else if (sensorType == ORB_SLAM2::System::STEREO)
  //   sync.registerCallback(&ImageGrabber::GrabStereo, &igb);

  rclcpp::spin(nodePtr);
  rclcpp::shutdown();
  return 0;
}
