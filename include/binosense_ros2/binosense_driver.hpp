#ifndef BINOSENSE_ROS2__BINOSENSE_DRIVER_HPP_
#define BINOSENSE_ROS2__BINOSENSE_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "binosense_ros2/binosense_device.hpp"
#include "binosense_ros2/msg/binosense_data.hpp"

// Service headers
#include "binosense_ros2/srv/set_motor_position.hpp"
#include "binosense_ros2/srv/set_motor_speed.hpp"
#include "binosense_ros2/srv/set_camera_exposure.hpp"
#include "binosense_ros2/srv/set_camera_white_balance.hpp"
#include "binosense_ros2/srv/set_camera_gain.hpp"
#include "binosense_ros2/srv/set_vor.hpp"
#include "binosense_ros2/srv/set_sv.hpp"
#include "binosense_ros2/srv/set_depth_control.hpp"
#include "binosense_ros2/srv/save_data.hpp"
#include "binosense_ros2/srv/set_image_resolution.hpp"
#include "binosense_ros2/srv/set_extra_module.hpp"
#include "binosense_ros2/srv/get_motor_limits.hpp"
#include "binosense_ros2/srv/set_neck_position.hpp"

namespace binosense_ros2
{

class BinosenseDriver : public rclcpp::Node
{
public:
  explicit BinosenseDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BinosenseDriver();

private:
  void timerCallback();

  // Topic callbacks
  void controlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Service callbacks
  void setMotorPositionSrv(
    const std::shared_ptr<binosense_ros2::srv::SetMotorPosition::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetMotorPosition::Response> response);

  void setMotorSpeedSrv(
    const std::shared_ptr<binosense_ros2::srv::SetMotorSpeed::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetMotorSpeed::Response> response);

  void setCameraExposureSrv(
    const std::shared_ptr<binosense_ros2::srv::SetCameraExposure::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetCameraExposure::Response> response);

  void setCameraWhiteBalanceSrv(
    const std::shared_ptr<binosense_ros2::srv::SetCameraWhiteBalance::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetCameraWhiteBalance::Response> response);

  void setCameraGainSrv(
    const std::shared_ptr<binosense_ros2::srv::SetCameraGain::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetCameraGain::Response> response);

  void setVORSrv(
    const std::shared_ptr<binosense_ros2::srv::SetVOR::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetVOR::Response> response);

  void setSVSrv(
    const std::shared_ptr<binosense_ros2::srv::SetSV::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetSV::Response> response);

  void setDepthControlSrv(
    const std::shared_ptr<binosense_ros2::srv::SetDepthControl::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetDepthControl::Response> response);

  void saveDataSrv(
    const std::shared_ptr<binosense_ros2::srv::SaveData::Request> request,
    std::shared_ptr<binosense_ros2::srv::SaveData::Response> response);

  void setImageResolutionSrv(
    const std::shared_ptr<binosense_ros2::srv::SetImageResolution::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetImageResolution::Response> response);

  void setExtraModuleSrv(
    const std::shared_ptr<binosense_ros2::srv::SetExtraModule::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetExtraModule::Response> response);

  void getMotorLimitsSrv(
    const std::shared_ptr<binosense_ros2::srv::GetMotorLimits::Request> request,
    std::shared_ptr<binosense_ros2::srv::GetMotorLimits::Response> response);

  void setNeckPositionSrv(
    const std::shared_ptr<binosense_ros2::srv::SetNeckPosition::Request> request,
    std::shared_ptr<binosense_ros2::srv::SetNeckPosition::Response> response);

  // Publishing helpers
  sensor_msgs::msg::CameraInfo getCameraInfo(
    const std::string & frame_id, int width, int height, bool is_left);

  void publishMotorState();
  void publishTF();
  void publishBinosenseData(const BE_GeneralData & data);
  void publishDepthAndPointCloud();

  std::unique_ptr<BinosenseDevice> device_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Image publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_pub_;

  // Depth / pointcloud publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_rectified_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_rectified_right_pub_;

  // State publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_state_pub_;
  rclcpp::Publisher<binosense_ros2::msg::BinosenseData>::SharedPtr binosense_data_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  // Services
  rclcpp::Service<binosense_ros2::srv::SetMotorPosition>::SharedPtr set_motor_position_srv_;
  rclcpp::Service<binosense_ros2::srv::SetMotorSpeed>::SharedPtr set_motor_speed_srv_;
  rclcpp::Service<binosense_ros2::srv::SetCameraExposure>::SharedPtr set_camera_exposure_srv_;
  rclcpp::Service<binosense_ros2::srv::SetCameraWhiteBalance>::SharedPtr set_camera_wb_srv_;
  rclcpp::Service<binosense_ros2::srv::SetCameraGain>::SharedPtr set_camera_gain_srv_;
  rclcpp::Service<binosense_ros2::srv::SetVOR>::SharedPtr set_vor_srv_;
  rclcpp::Service<binosense_ros2::srv::SetSV>::SharedPtr set_sv_srv_;
  rclcpp::Service<binosense_ros2::srv::SetDepthControl>::SharedPtr set_depth_control_srv_;
  rclcpp::Service<binosense_ros2::srv::SaveData>::SharedPtr save_data_srv_;
  rclcpp::Service<binosense_ros2::srv::SetImageResolution>::SharedPtr set_image_resolution_srv_;
  rclcpp::Service<binosense_ros2::srv::SetExtraModule>::SharedPtr set_extra_module_srv_;
  rclcpp::Service<binosense_ros2::srv::GetMotorLimits>::SharedPtr get_motor_limits_srv_;
  rclcpp::Service<binosense_ros2::srv::SetNeckPosition>::SharedPtr set_neck_position_srv_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string frame_prefix_;
  std::string connection_mode_;
  std::string device_ip_;
  double publish_rate_;
  bool publish_tf_;
  bool stereo_enabled_;
  bool depth_enabled_;
  int depth_precision_;
  float depth_min_;
  float depth_max_;
  bool depth_sv_enabled_;

  sensor_msgs::msg::CameraInfo left_camera_info_;
  sensor_msgs::msg::CameraInfo right_camera_info_;

  float current_positions_[6];
  float target_positions_[6];
};

}  // namespace binosense_ros2

#endif  // BINOSENSE_ROS2__BINOSENSE_DRIVER_HPP_
