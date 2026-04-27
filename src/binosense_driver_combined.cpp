#include "binosense_ros2/binosense_device.hpp"
#include "binosense_ros2/binosense_driver.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace binosense_ros2
{

// ============================================================================
// BinosenseDevice implementation
// ============================================================================

BinosenseDevice::BinosenseDevice()
: current_frame_id_(0),
  current_timestamp_(0),
  connected_(false)
{
  memset(&current_data_, 0, sizeof(BE_GeneralData));
}

BinosenseDevice::~BinosenseDevice()
{
  disconnect();
}

bool BinosenseDevice::connect(const std::string & mode, const std::string & ip)
{
  if (connected_) {
    disconnect();
  }

  BE_Connect_Type connect_type;
  if (mode == "i" || mode == "image") {
    connect_type = enumConnect_Image;
  } else if (mode == "c" || mode == "control") {
    connect_type = enumConnect_Control;
  } else {
    connect_type = enumConnect_ImageControl;
  }

  try {
    if (ip.empty()) {
      device_ = std::make_unique<bionic_eyes::BionicEyesWrapper>(
        connect_type, enumDeviceServer_Only, enumDataTransmission_ASPAP);
    } else {
      device_ = std::make_unique<bionic_eyes::BionicEyesWrapper>(
        ip, connect_type, enumDeviceServer_Only, enumDataTransmission_ASPAP);
    }

    device_->setImageColor(enumColor);
    device_->setImageColor_Transfer(enumColor);

    connected_ = true;

    int timeout = 100;
    while (timeout > 0 && !device_->isBeDataReady()) {
      usleep(10000);
      timeout--;
    }

    return true;
  } catch (const std::exception & e) {
    connected_ = false;
    return false;
  }
}

void BinosenseDevice::disconnect()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (device_) {
    device_.reset();
  }
  connected_ = false;
}

bool BinosenseDevice::isConnected() const
{
  return connected_ && device_ != nullptr;
}

bool BinosenseDevice::isDataReady()
{
  if (!connected_ || !device_) {
    return false;
  }
  return device_->isBeDataReady();
}

bool BinosenseDevice::getImageData(EyeSide side, BE_Image & image)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!connected_ || !device_) {
    return false;
  }

  device_->getBeData(current_data_);
  current_frame_id_ = current_data_.frame_id;
  current_timestamp_ = current_data_.timestamp;

  int image_idx = (side == RIGHT_EYE) ? enumRight :
                   (side == LEFT_EYE) ? enumLeft : enumBoth;

  if (current_data_.Image_data[image_idx].width > 0 &&
      current_data_.Image_data[image_idx].data != nullptr)
  {
    memcpy(&image, &current_data_.Image_data[image_idx], sizeof(BE_Image));
    return true;
  }

  return false;
}

bool BinosenseDevice::getStereoImages(BE_Image & left, BE_Image & right)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!connected_ || !device_) {
    return false;
  }

  device_->getBeData(current_data_);
  current_frame_id_ = current_data_.frame_id;
  current_timestamp_ = current_data_.timestamp;

  if (current_data_.Image_data[enumBoth].width > 0) {
    BE_Image & both = current_data_.Image_data[enumBoth];
    int half_width = both.width / 2;

    right.width = half_width;
    right.height = both.height;
    right.channels = both.channels;
    right.format = both.format;
    right.data_size = both.data_size / 2;
    right.data = both.data;

    left.width = half_width;
    left.height = both.height;
    left.channels = both.channels;
    left.format = both.format;
    left.data_size = both.data_size / 2;
    left.data = both.data + left.data_size;

    return true;
  }

  if (current_data_.Image_data[enumLeft].width > 0 &&
      current_data_.Image_data[enumRight].width > 0)
  {
    memcpy(&left, &current_data_.Image_data[enumLeft], sizeof(BE_Image));
    memcpy(&right, &current_data_.Image_data[enumRight], sizeof(BE_Image));
    return true;
  }

  return false;
}

bool BinosenseDevice::getMotorState(MotorState & state)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!connected_ || !device_) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    state.position[i] = current_data_.motorData[i];
    state.speed[i] = 0.0f;
    state.enabled[i] = true;
  }

  return true;
}

bool BinosenseDevice::getCameraState(CameraState & state)
{
  if (!connected_ || !device_) {
    return false;
  }

  state.exposure_time[0] = device_->getCameraExposureTime(enumRight);
  state.exposure_time[1] = device_->getCameraExposureTime(enumLeft);
  state.white_balance[0] = device_->getCameraWhiteBalanceTemperature(enumRight);
  state.white_balance[1] = device_->getCameraWhiteBalanceTemperature(enumLeft);

  return true;
}

bool BinosenseDevice::setMotorPositions(
  const float positions[6],
  BE_MovePatternType pattern,
  BE_MoveBaseType base)
{
  if (!connected_ || !device_) {
    return false;
  }

  device_->setAbsolutePosition(enumAllMotor, const_cast<float *>(positions), pattern, base);
  return true;
}

bool BinosenseDevice::setMotorRelativePositions(
  const float positions[6],
  BE_MovePatternType pattern,
  BE_MoveBaseType base)
{
  if (!connected_ || !device_) {
    return false;
  }

  device_->setRelativePosition(enumAllMotor, const_cast<float *>(positions), pattern, base);
  return true;
}

bool BinosenseDevice::setMotorSpeeds(const float speeds[6], const bool active[6])
{
  if (!connected_ || !device_) {
    return false;
  }

  device_->setAbsoluteSpeed(const_cast<bool *>(active), const_cast<float *>(speeds));
  return true;
}

bool BinosenseDevice::goHome()
{
  if (!connected_ || !device_) {
    return false;
  }

  device_->goInitPosition(enumAllMotor);
  return true;
}

bool BinosenseDevice::setCurrentAsHome()
{
  if (!connected_ || !device_) {
    return false;
  }

  device_->setNowPositionAsInitPos();
  return true;
}

bool BinosenseDevice::setExposure(EyeSide side, bool auto_exposure, float exposure_time)
{
  if (!connected_ || !device_) {
    return false;
  }

  int camera_type = (side == BOTH_EYES) ? enumBoth :
                    (side == RIGHT_EYE) ? enumRight : enumLeft;

  device_->setCameraExposure(camera_type, auto_exposure, exposure_time);
  return true;
}

bool BinosenseDevice::setWhiteBalance(EyeSide side, bool auto_wb, float temperature)
{
  if (!connected_ || !device_) {
    return false;
  }

  int camera_type = (side == BOTH_EYES) ? enumBoth :
                    (side == RIGHT_EYE) ? enumRight : enumLeft;

  device_->setCameraWhiteBalanceTemperature(camera_type, auto_wb, temperature);
  return true;
}

bool BinosenseDevice::setGain(EyeSide side, bool auto_gain, int gain)
{
  if (!connected_ || !device_) {
    return false;
  }

  int camera_type = (side == BOTH_EYES) ? enumBoth :
                    (side == RIGHT_EYE) ? enumRight : enumLeft;

  device_->setCameraGain(camera_type, auto_gain, gain);
  return true;
}

void BinosenseDevice::setVOR(bool vor_eye, bool vor_neck)
{
  if (connected_ && device_) {
    device_->onoff_VOR(vor_eye, vor_neck);
  }
}

void BinosenseDevice::setSV(bool onoff)
{
  if (connected_ && device_) {
    device_->onoff_SV(onoff);
  }
}

bool BinosenseDevice::setDepthControl(bool onOff, bool svOnOff, int precision, float min, float max)
{
  if (!connected_ || !device_) {
    return false;
  }
  device_->setDepthControl(onOff, svOnOff, precision, min, max);
  return true;
}

bool BinosenseDevice::getDepthInfo(DepthData & depth)
{
  if (!connected_ || !device_) {
    return false;
  }
  memset(&depth.bgr_left, 0, sizeof(BE_Image));
  return device_->getDepthInfo(depth.id, depth.Z, depth.L0LnRT, depth.KLrect, depth.bgr_left);
}

bool BinosenseDevice::getNewestDepthInfo(DepthData & depth)
{
  if (!connected_ || !device_) {
    return false;
  }
  memset(&depth.bgr_left, 0, sizeof(BE_Image));
  return device_->getNewestDepthInfo(depth.id, depth.Z, depth.L0LnRT, depth.KLrect, depth.bgr_left);
}

bool BinosenseDevice::getMotorLimits(BE_MotorType type, float & up, float & down)
{
  if (!connected_ || !device_) {
    return false;
  }
  std::pair<float, float> limits = device_->getUpDownLimit(type);
  up = limits.first;
  down = limits.second;
  return true;
}

bool BinosenseDevice::getNeckMotorLimits(BE_MotorType_Neck type, float & up, float & down)
{
  if (!connected_ || !device_) {
    return false;
  }
  std::pair<float, float> limits = device_->getUpDownLimit_Neck(type);
  up = limits.first;
  down = limits.second;
  return true;
}

bool BinosenseDevice::hasNeck()
{
  if (!connected_ || !device_) {
    return false;
  }
  return device_->haveNeckLinked();
}

bool BinosenseDevice::setNeckAbsolutePosition(const bool activeFlag[3], const float angle[3], BE_MovePatternType moveType)
{
  if (!connected_ || !device_) {
    return false;
  }
  device_->setAbsolutePosition_Neck(const_cast<bool *>(activeFlag), const_cast<float *>(angle), moveType);
  return true;
}

bool BinosenseDevice::setNeckRelativePosition(const bool activeFlag[3], const float angle[3], BE_MovePatternType moveType)
{
  if (!connected_ || !device_) {
    return false;
  }
  device_->setRelativePosition_Neck(const_cast<bool *>(activeFlag), const_cast<float *>(angle), moveType);
  return true;
}

bool BinosenseDevice::setUnionAbsolutePositionNeckEye(const bool activeFlag[3],
    const float eyeAngle[6], const float neckAngle[3], BE_MovePatternType moveType)
{
  if (!connected_ || !device_) {
    return false;
  }
  device_->setUnionAbsolutePosition_NeckEye(
    const_cast<bool *>(activeFlag), const_cast<float *>(eyeAngle),
    const_cast<float *>(neckAngle), moveType);
  return true;
}

bool BinosenseDevice::setUnionRelativePositionNeckEye(const bool activeFlag[3],
    const float eyeAngle[6], const float neckAngle[3], BE_MovePatternType moveType)
{
  if (!connected_ || !device_) {
    return false;
  }
  device_->setUnionRelativePosition_NeckEye(
    const_cast<bool *>(activeFlag), const_cast<float *>(eyeAngle),
    const_cast<float *>(neckAngle), moveType);
  return true;
}

void BinosenseDevice::goNeckInitPosition(BE_MotorType_Neck type)
{
  if (connected_ && device_) {
    device_->goInitPosition_Neck(type);
  }
}

void BinosenseDevice::setImageResolution(int width, int height)
{
  if (connected_ && device_) {
    device_->setImageResolution(width, height);
  }
}

void BinosenseDevice::setImageResolutionTransfer(int width, int height)
{
  if (connected_ && device_) {
    device_->setImageResolution_Transfer(width, height);
  }
}

void BinosenseDevice::setImageColor(BE_ImageColorType type)
{
  if (connected_ && device_) {
    device_->setImageColor(type);
  }
}

void BinosenseDevice::setImageColorTransfer(BE_ImageColorType type)
{
  if (connected_ && device_) {
    device_->setImageColor_Transfer(type);
  }
}

void BinosenseDevice::setDataRateTransfer(float rate)
{
  if (connected_ && device_) {
    device_->setDataRate_Transfer(rate);
  }
}

void BinosenseDevice::setImageCompressionQuality(int quality)
{
  if (connected_ && device_) {
    device_->setImageCompressionQuality(quality);
  }
}

BE_Size BinosenseDevice::getOriginImageResolution()
{
  if (connected_ && device_) {
    std::pair<int, int> size = device_->getOriginImageResolution();
    BE_Size sz;
    sz.width = size.first;
    sz.height = size.second;
    return sz;
  }
  BE_Size sz = {0, 0};
  return sz;
}

float BinosenseDevice::getMaxImageFrameRate()
{
  if (connected_ && device_) {
    return device_->getMaxImageFrameRate();
  }
  return 0.0f;
}

void BinosenseDevice::setBeDataRate(int rate)
{
  if (connected_ && device_) {
    device_->setBeDataRate(rate);
  }
}

void BinosenseDevice::setSyncCameraDelayTime(int timeDelay)
{
  if (connected_ && device_) {
    device_->setSyncCameraDelayTime(timeDelay);
  }
}

void BinosenseDevice::triggerDataTransmission()
{
  if (connected_ && device_) {
    device_->triggerDataTransmission();
  }
}

void BinosenseDevice::saveBeData(bool startStop, const std::string & folderPath)
{
  if (connected_ && device_) {
    device_->saveBeData(startStop, folderPath);
  }
}

bool BinosenseDevice::snapBeData(const std::string & description, const std::string & folderPath)
{
  if (!connected_ || !device_) {
    return false;
  }
  return device_->snapBeData(description, folderPath);
}

void BinosenseDevice::stopSnapBeData()
{
  if (connected_ && device_) {
    device_->stopSnapBeData();
  }
}

bool BinosenseDevice::recordBeData(BE_GeneralData * data, const std::string & description, const std::string & folderPath)
{
  if (!connected_ || !device_) {
    return false;
  }
  return device_->recordBeData(*data, description, folderPath);
}

void BinosenseDevice::stopRecordBeData()
{
  if (connected_ && device_) {
    device_->stopRecordBeData();
  }
}

void BinosenseDevice::setExtraModuleFunction(BE_ExtraModuleType type, int para0, int para1, int para2)
{
  if (connected_ && device_) {
    device_->setExtraModuleFunction(type, para0, para1, para2);
  }
}

void BinosenseDevice::setZoomFocalLength(uint8_t focalLength)
{
  if (connected_ && device_) {
    device_->setZoomFocalLength(focalLength);
  }
}

void BinosenseDevice::setWipers(uint8_t value)
{
  if (connected_ && device_) {
    device_->setWipers(value);
  }
}

void BinosenseDevice::setMotorPositionSpeed(int saccadeSpeed, int pursuitSpeed)
{
  if (connected_ && device_) {
    device_->setMotorPositionSpeed(saccadeSpeed, pursuitSpeed);
  }
}

void BinosenseDevice::setSyncSignalSource(bool flag)
{
  if (connected_ && device_) {
    device_->setSyncSignalSource(flag);
  }
}

void BinosenseDevice::onOffCameraImage(bool flag[], bool onlyNetworkType)
{
  if (connected_ && device_) {
    device_->onOffCameraImage(flag, onlyNetworkType);
  }
}

uint32_t BinosenseDevice::getDeviceIp()
{
  if (connected_ && device_) {
    return device_->getBeDevice_Ip();
  }
  return 0;
}

const char* BinosenseDevice::getDeviceIpStr()
{
  if (connected_ && device_) {
    ip_str_buffer_ = device_->getBeDevice_Ip_str();
    return ip_str_buffer_.c_str();
  }
  return "";
}

BE_Device_Type BinosenseDevice::getDeviceType()
{
  if (connected_ && device_) {
    return device_->getBeDeviceType();
  }
  return enumUnknownDevice;
}

// ============================================================================
// BinosenseDriver implementation
// ============================================================================

BinosenseDriver::BinosenseDriver(const rclcpp::NodeOptions & options)
: Node("binosense_driver", options),
  device_(std::make_unique<BinosenseDevice>()),
  frame_prefix_("binosense"),
  connection_mode_("ic"),
  publish_rate_(25.0),
  publish_tf_(true),
  stereo_enabled_(true),
  depth_enabled_(false),
  depth_precision_(1),
  depth_min_(200.0f),
  depth_max_(5000.0f),
  depth_sv_enabled_(false)
{
  this->declare_parameter<std::string>("frame_prefix", frame_prefix_);
  this->declare_parameter<std::string>("connection_mode", connection_mode_);
  this->declare_parameter<std::string>("device_ip", "");
  this->declare_parameter<double>("publish_rate", publish_rate_);
  this->declare_parameter<bool>("publish_tf", publish_tf_);
  this->declare_parameter<bool>("stereo_enabled", stereo_enabled_);
  this->declare_parameter<bool>("depth_enabled", depth_enabled_);
  this->declare_parameter<int>("depth_precision", depth_precision_);
  this->declare_parameter<float>("depth_min", depth_min_);
  this->declare_parameter<float>("depth_max", depth_max_);
  this->declare_parameter<bool>("depth_sv_enabled", depth_sv_enabled_);
  this->declare_parameter<std::string>("camera_info_url", "");

  this->get_parameter("frame_prefix", frame_prefix_);
  this->get_parameter("connection_mode", connection_mode_);
  this->get_parameter("device_ip", device_ip_);
  this->get_parameter("publish_rate", publish_rate_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("stereo_enabled", stereo_enabled_);
  this->get_parameter("depth_enabled", depth_enabled_);
  this->get_parameter("depth_precision", depth_precision_);
  this->get_parameter("depth_min", depth_min_);
  this->get_parameter("depth_max", depth_max_);
  this->get_parameter("depth_sv_enabled", depth_sv_enabled_);

  // Image publishers
  left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_raw", 1);
  right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_raw", 1);
  left_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
  right_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);

  // Depth / pointcloud publishers
  depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 1);
  depth_image_color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_color", 1);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("depth/points", 1);
  depth_rectified_left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/rectified_left/image_raw", 1);
  depth_rectified_right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/rectified_right/image_raw", 1);

  // State publishers
  motor_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_state", 10);
  binosense_data_pub_ = this->create_publisher<binosense_ros2::msg::BinosenseData>("binosense_data", 10);

  // Subscriptions
  motor_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "motor_command", 10,
    std::bind(&BinosenseDriver::controlCallback, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&BinosenseDriver::twistCallback, this, std::placeholders::_1));

  // Services
  set_motor_position_srv_ = this->create_service<binosense_ros2::srv::SetMotorPosition>(
    "set_motor_position",
    std::bind(&BinosenseDriver::setMotorPositionSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_motor_speed_srv_ = this->create_service<binosense_ros2::srv::SetMotorSpeed>(
    "set_motor_speed",
    std::bind(&BinosenseDriver::setMotorSpeedSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_camera_exposure_srv_ = this->create_service<binosense_ros2::srv::SetCameraExposure>(
    "set_camera_exposure",
    std::bind(&BinosenseDriver::setCameraExposureSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_camera_wb_srv_ = this->create_service<binosense_ros2::srv::SetCameraWhiteBalance>(
    "set_camera_white_balance",
    std::bind(&BinosenseDriver::setCameraWhiteBalanceSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_camera_gain_srv_ = this->create_service<binosense_ros2::srv::SetCameraGain>(
    "set_camera_gain",
    std::bind(&BinosenseDriver::setCameraGainSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_vor_srv_ = this->create_service<binosense_ros2::srv::SetVOR>(
    "set_vor",
    std::bind(&BinosenseDriver::setVORSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_sv_srv_ = this->create_service<binosense_ros2::srv::SetSV>(
    "set_sv",
    std::bind(&BinosenseDriver::setSVSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_depth_control_srv_ = this->create_service<binosense_ros2::srv::SetDepthControl>(
    "set_depth_control",
    std::bind(&BinosenseDriver::setDepthControlSrv, this, std::placeholders::_1, std::placeholders::_2));

  save_data_srv_ = this->create_service<binosense_ros2::srv::SaveData>(
    "save_data",
    std::bind(&BinosenseDriver::saveDataSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_image_resolution_srv_ = this->create_service<binosense_ros2::srv::SetImageResolution>(
    "set_image_resolution",
    std::bind(&BinosenseDriver::setImageResolutionSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_extra_module_srv_ = this->create_service<binosense_ros2::srv::SetExtraModule>(
    "set_extra_module",
    std::bind(&BinosenseDriver::setExtraModuleSrv, this, std::placeholders::_1, std::placeholders::_2));

  get_motor_limits_srv_ = this->create_service<binosense_ros2::srv::GetMotorLimits>(
    "get_motor_limits",
    std::bind(&BinosenseDriver::getMotorLimitsSrv, this, std::placeholders::_1, std::placeholders::_2));

  set_neck_position_srv_ = this->create_service<binosense_ros2::srv::SetNeckPosition>(
    "set_neck_position",
    std::bind(&BinosenseDriver::setNeckPositionSrv, this, std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  for (int i = 0; i < 6; i++) {
    current_positions_[i] = 0.0f;
    target_positions_[i] = 0.0f;
  }

  RCLCPP_INFO(this->get_logger(), "Connecting to Binosense device (IP: %s)...",
    device_ip_.empty() ? "auto-detect" : device_ip_.c_str());

  if (device_->connect(connection_mode_, device_ip_)) {
    RCLCPP_INFO(this->get_logger(), "Successfully connected to Binosense device");
    device_->setVOR(false, false);
    device_->setSV(false);
    device_->goHome();

    if (depth_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Enabling depth: precision=%d, range=[%.1f, %.1f], sv=%s",
        depth_precision_, depth_min_, depth_max_, depth_sv_enabled_ ? "on" : "off");
      device_->setDepthControl(true, depth_sv_enabled_, depth_precision_, depth_min_, depth_max_);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to connect to device. Running in simulation mode.");
  }

  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&BinosenseDriver::timerCallback, this));
}

BinosenseDriver::~BinosenseDriver()
{
  if (device_) {
    device_->disconnect();
  }
}

void BinosenseDriver::timerCallback()
{
  if (!device_->isConnected()) {
    return;
  }

  if (!device_->isDataReady()) {
    return;
  }

  BE_GeneralData be_data = {0};

  if (device_->getBeDataRaw(be_data)) {
    publishBinosenseData(be_data);

    if (stereo_enabled_) {
      cv::Mat imgL_temp, imgR_temp;
      cv::Mat imgL_raw, imgR_raw;

      if (be_data.Image_data[enumBoth].width > 0) {
        cv::Mat org = ImageConverter::toMat(&be_data.Image_data[enumBoth], false);
        int w = org.cols / 2;
        imgL_temp = org.colRange(0, w);
        imgR_temp = org.colRange(w, 2 * w);

        imgL_raw = imgL_temp.clone();
        imgR_raw = imgR_temp.clone();

        std_msgs::msg::Header header_left;
        header_left.stamp = this->now();
        header_left.frame_id = frame_prefix_ + "/left_camera";
        auto img_msg_left = cv_bridge::CvImage(header_left, "bgr8", imgL_raw).toImageMsg();
        auto cam_info_left = getCameraInfo(frame_prefix_ + "/left_camera", imgL_raw.cols, imgL_raw.rows, true);
        left_image_pub_->publish(*img_msg_left);
        left_camera_info_pub_->publish(cam_info_left);

        std_msgs::msg::Header header_right;
        header_right.stamp = this->now();
        header_right.frame_id = frame_prefix_ + "/right_camera";
        auto img_msg_right = cv_bridge::CvImage(header_right, "bgr8", imgR_raw).toImageMsg();
        auto cam_info_right = getCameraInfo(frame_prefix_ + "/right_camera", imgR_raw.cols, imgR_raw.rows, false);
        right_image_pub_->publish(*img_msg_right);
        right_camera_info_pub_->publish(cam_info_right);
      } else if (be_data.Image_data[enumLeft].width > 0 && be_data.Image_data[enumRight].width > 0) {
        imgL_temp = ImageConverter::toMat(&be_data.Image_data[enumLeft], false);
        imgR_temp = ImageConverter::toMat(&be_data.Image_data[enumRight], false);

        imgL_raw = imgL_temp.clone();
        imgR_raw = imgR_temp.clone();

        std_msgs::msg::Header header_left;
        header_left.stamp = this->now();
        header_left.frame_id = frame_prefix_ + "/left_camera";
        auto img_msg_left = cv_bridge::CvImage(header_left, "bgr8", imgL_raw).toImageMsg();
        auto cam_info_left = getCameraInfo(frame_prefix_ + "/left_camera", imgL_raw.cols, imgL_raw.rows, true);
        left_image_pub_->publish(*img_msg_left);
        left_camera_info_pub_->publish(cam_info_left);

        std_msgs::msg::Header header_right;
        header_right.stamp = this->now();
        header_right.frame_id = frame_prefix_ + "/right_camera";
        auto img_msg_right = cv_bridge::CvImage(header_right, "bgr8", imgR_raw).toImageMsg();
        auto cam_info_right = getCameraInfo(frame_prefix_ + "/right_camera", imgR_raw.cols, imgR_raw.rows, false);
        right_image_pub_->publish(*img_msg_right);
        right_camera_info_pub_->publish(cam_info_right);
      }
    }

    // Depth + pointcloud
    if (depth_enabled_) {
      publishDepthAndPointCloud();
    }
  }

  publishMotorState();
  if (publish_tf_) {
    publishTF();
  }
}

void BinosenseDriver::publishDepthAndPointCloud()
{
  DepthData depth;
  if (!device_->getNewestDepthInfo(depth)) {
    return;
  }

  if (depth.bgr_left.width == 0 || depth.Z.empty()) {
    return;
  }

  int w = 640, h = 480;
  if (depth.bgr_left.width > 0) {
    w = depth.bgr_left.width;
    h = depth.bgr_left.height;
  }

  // Publish rectified left image
  try {
    cv::Mat cv_bgr = ImageConverter::toMat(&depth.bgr_left, false);
    if (!cv_bgr.empty()) {
      std_msgs::msg::Header hdr;
      hdr.stamp = this->now();
      hdr.frame_id = frame_prefix_ + "/left_camera";
      auto img_msg = cv_bridge::CvImage(hdr, "bgr8", cv_bgr).toImageMsg();
      depth_rectified_left_pub_->publish(*img_msg);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Depth image conversion error: %s", e.what());
  }

  // Publish depth image as 16UC1 (mm units)
  {
    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id = frame_prefix_ + "/left_camera";

    sensor_msgs::msg::Image depth_msg;
    depth_msg.header = hdr;
    depth_msg.height = h;
    depth_msg.width = w;
    depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
    depth_msg.is_bigendian = false;
    depth_msg.step = w * 2;
    depth_msg.data.resize(w * h * 2);

    if (static_cast<int>(depth.Z.size()) >= w * h) {
      memcpy(depth_msg.data.data(), depth.Z.data(), w * h * 2);
    }

    depth_image_pub_->publish(depth_msg);

    // Publish colorized depth using JET colormap
    try {
      cv::Mat depth_mat(h, w, CV_16UC1, depth.Z.data());
      cv::Mat depth_normalized;

      // Normalize depth values to 0-255 range
      cv::Mat depth_float;
      depth_mat.convertTo(depth_float, CV_32F);

      // Mask out invalid values
      cv::Mat valid_mask = (depth_float >= depth_min_) & (depth_float <= depth_max_);

      // Normalize valid range to 0-255
      double alpha = 255.0 / (depth_max_ - depth_min_);
      double beta = -depth_min_ * alpha;

      cv::Mat depth_8u;
      depth_float.convertTo(depth_8u, CV_8U, alpha, beta);

      // Set invalid areas to 0 (black)
      depth_8u.setTo(0, ~valid_mask);

      // Apply JET colormap
      cv::Mat depth_color;
      cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_JET);

      // Set invalid areas to black (overwrite colormap)
      depth_color.setTo(cv::Scalar(0, 0, 0), ~valid_mask);

      std_msgs::msg::Header hdr;
      hdr.stamp = this->now();
      hdr.frame_id = frame_prefix_ + "/left_camera";
      auto color_msg = cv_bridge::CvImage(hdr, "bgr8", depth_color).toImageMsg();
      depth_image_color_pub_->publish(*color_msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Colorized depth conversion error: %s", e.what());
    }
  }

  // Publish rectified right image (from raw right image)
  try {
    BE_Image right_img;
    if (device_->getImageData(BinosenseDevice::RIGHT_EYE, right_img)) {
      cv::Mat cv_right = ImageConverter::toMat(&right_img, false);
      if (!cv_right.empty()) {
        std_msgs::msg::Header hdr;
        hdr.stamp = this->now();
        hdr.frame_id = frame_prefix_ + "/right_camera";
        auto img_msg = cv_bridge::CvImage(hdr, "bgr8", cv_right).toImageMsg();
        depth_rectified_right_pub_->publish(*img_msg);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Right depth image conversion error: %s", e.what());
  }

  // Build and publish PointCloud2
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = frame_prefix_ + "/base_link";
    cloud_msg.height = 1;
    cloud_msg.width = 0;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(w * h);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    cv::Mat cv_bgr = ImageConverter::toMat(&depth.bgr_left, false);

    // Convert Z to XYZ in 0-position
    std::vector<float> XYZ(w * h * 3);
    for (int i = 0; i < h; i++) {
      for (int j = 0; j < w; j++) {
        int idx = i * w + j;
        unsigned short zz = depth.Z[idx];
        if (zz == std::numeric_limits<uint16_t>::max() || zz < depth_min_ || zz > depth_max_) {
          XYZ[idx * 3] = std::numeric_limits<float>::quiet_NaN();
          XYZ[idx * 3 + 1] = std::numeric_limits<float>::quiet_NaN();
          XYZ[idx * 3 + 2] = std::numeric_limits<float>::quiet_NaN();
        } else {
          float z_mm = static_cast<float>(zz) / depth_precision_;
          float fx = depth.KLrect[0];
          float fy = depth.KLrect[4];
          float cx = depth.KLrect[2];
          float cy = depth.KLrect[5];
          XYZ[idx * 3] = (j - cx) * z_mm / fx;
          XYZ[idx * 3 + 1] = (i - cy) * z_mm / fy;
          XYZ[idx * 3 + 2] = z_mm;
        }
      }
    }

    // Transform to 0-position if we have RT
    if (!depth.L0LnRT.empty() && depth.L0LnRT.size() >= 6) {
      cv::Mat r_vec(3, 1, CV_32FC1, depth.L0LnRT.data());
      cv::Mat t_vec(3, 1, CV_32FC1, depth.L0LnRT.data() + 3);
      cv::Mat r_mat;
      cv::Rodrigues(r_vec, r_mat);
      cv::Mat r_inv = r_mat.t();
      cv::Mat t_inv = -r_inv * t_vec;

      for (int i = 0; i < w * h; i++) {
        if (!std::isfinite(XYZ[i * 3])) continue;

        cv::Mat p(3, 1, CV_32FC1, &XYZ[i * 3]);
        cv::Mat p0 = r_inv * p + t_inv;

        XYZ[i * 3] = p0.at<float>(0);
        XYZ[i * 3 + 1] = p0.at<float>(1);
        XYZ[i * 3 + 2] = p0.at<float>(2);
      }
    }

    // Fill PointCloud2
    size_t valid_count = 0;
    for (int i = 0; i < w * h; i++) {
      if (!std::isfinite(XYZ[i * 3]) || !std::isfinite(XYZ[i * 3 + 1]) || !std::isfinite(XYZ[i * 3 + 2])) {
        continue;
      }

      // Convert mm to meters
      *iter_x = XYZ[i * 3] / 1000.0f;
      *iter_y = XYZ[i * 3 + 1] / 1000.0f;
      *iter_z = XYZ[i * 3 + 2] / 1000.0f;

      if (!cv_bgr.empty()) {
        cv::Vec3b color = cv_bgr.at<cv::Vec3b>(i / w, i % w);
        *iter_r = color[2];
        *iter_g = color[1];
        *iter_b = color[0];
      } else {
        *iter_r = 128;
        *iter_g = 128;
        *iter_b = 128;
      }

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
      valid_count++;
    }

    modifier.resize(valid_count);
    cloud_msg.width = valid_count;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    pointcloud_pub_->publish(cloud_msg);
  }

  // Free depth image memory
  BE_freeImage(&depth.bgr_left);
}

void BinosenseDriver::publishBinosenseData(const BE_GeneralData & data)
{
  auto msg = binosense_ros2::msg::BinosenseData();

  msg.header.stamp = this->now();
  msg.header.frame_id = frame_prefix_ + "/base_link";

  msg.frame_id = data.frame_id;
  msg.timestamp = data.timestamp;

  // Handle SBS image
  if (data.Image_data[enumBoth].width > 0) {
    cv::Mat imgBoth = ImageConverter::toMat(const_cast<BE_Image *>(&data.Image_data[enumBoth]), false);
    if (!imgBoth.empty()) {
      int w = imgBoth.cols / 2;
      cv::Mat imgL = imgBoth.colRange(0, w).clone();
      cv::Mat imgR = imgBoth.colRange(w, 2 * w).clone();

      std_msgs::msg::Header hdr_l;
      hdr_l.stamp = this->now();
      hdr_l.frame_id = frame_prefix_ + "/left_camera";
      msg.left_image = *cv_bridge::CvImage(hdr_l, "bgr8", imgL).toImageMsg();
      msg.left_camera_info = getCameraInfo(frame_prefix_ + "/left_camera", imgL.cols, imgL.rows, true);

      std_msgs::msg::Header hdr_r;
      hdr_r.stamp = this->now();
      hdr_r.frame_id = frame_prefix_ + "/right_camera";
      msg.right_image = *cv_bridge::CvImage(hdr_r, "bgr8", imgR).toImageMsg();
      msg.right_camera_info = getCameraInfo(frame_prefix_ + "/right_camera", imgR.cols, imgR.rows, false);

      std_msgs::msg::Header hdr_b;
      hdr_b.stamp = this->now();
      hdr_b.frame_id = frame_prefix_ + "/both_camera";
      msg.both_image = *cv_bridge::CvImage(hdr_b, "bgr8", imgBoth).toImageMsg();
    }
  } else {
    if (data.Image_data[enumLeft].width > 0) {
      cv::Mat imgL = ImageConverter::toMat(const_cast<BE_Image *>(&data.Image_data[enumLeft]), false);
      if (!imgL.empty()) {
        std_msgs::msg::Header hdr_l;
        hdr_l.stamp = this->now();
        hdr_l.frame_id = frame_prefix_ + "/left_camera";
        msg.left_image = *cv_bridge::CvImage(hdr_l, "bgr8", imgL).toImageMsg();
        msg.left_camera_info = getCameraInfo(frame_prefix_ + "/left_camera", imgL.cols, imgL.rows, true);
      }
    }

    if (data.Image_data[enumRight].width > 0) {
      cv::Mat imgR = ImageConverter::toMat(const_cast<BE_Image *>(&data.Image_data[enumRight]), false);
      if (!imgR.empty()) {
        std_msgs::msg::Header hdr_r;
        hdr_r.stamp = this->now();
        hdr_r.frame_id = frame_prefix_ + "/right_camera";
        msg.right_image = *cv_bridge::CvImage(hdr_r, "bgr8", imgR).toImageMsg();
        msg.right_camera_info = getCameraInfo(frame_prefix_ + "/right_camera", imgR.cols, imgR.rows, false);
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    msg.motor_data[i] = data.motorData[i];
  }

  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 4; j++) {
      msg.imu_data[i * 4 + j] = data.imuData[i][j];
    }
  }

  msg.gps_data[0] = data.gpsData[0];
  msg.gps_data[1] = data.gpsData[1];
  msg.gps_data[2] = data.gpsData[2];
  msg.gps_time = data.gpsTime;
  msg.is_moving_fastly = data.isMovingFastly;

  binosense_data_pub_->publish(msg);
}

sensor_msgs::msg::CameraInfo BinosenseDriver::getCameraInfo(
  const std::string & frame_id,
  int width, int height, bool is_left)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.header.stamp = this->now();
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";

  double fx = 500.0;
  double fy = 500.0;
  double cx = width / 2.0;
  double cy = height / 2.0;

  info.k[0] = fx; info.k[2] = cx;
  info.k[4] = fy; info.k[5] = cy;
  info.k[8] = 1.0;

  info.p[0] = fx; info.p[2] = cx;
  info.p[5] = fy; info.p[6] = cy;
  info.p[10] = 1.0;

  if (!is_left) {
    info.p[3] = -fx * 0.06;
  }

  info.d.resize(5, 0.0);
  info.r[0] = info.r[4] = info.r[8] = 1.0;

  return info;
}

void BinosenseDriver::publishMotorState()
{
  MotorState state;
  if (device_->getMotorState(state)) {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.reserve(6);
    for (int i = 0; i < 6; i++) {
      msg.data.push_back(state.position[i]);
      current_positions_[i] = state.position[i];
    }
    motor_state_pub_->publish(msg);
  }
}

void BinosenseDriver::publishTF()
{
  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp = this->now();
  tf.header.frame_id = frame_prefix_ + "/base_link";
  tf.child_frame_id = frame_prefix_ + "/left_camera";

  tf.transform.translation.x = 0.03;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(current_positions_[3] * M_PI / 180.0,
           current_positions_[4] * M_PI / 180.0,
           current_positions_[5] * M_PI / 180.0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);

  tf.header.frame_id = frame_prefix_ + "/base_link";
  tf.child_frame_id = frame_prefix_ + "/right_camera";

  tf.transform.translation.x = -0.03;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;

  q.setRPY(current_positions_[0] * M_PI / 180.0,
           current_positions_[1] * M_PI / 180.0,
           current_positions_[2] * M_PI / 180.0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);
}

void BinosenseDriver::controlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 6) {
    RCLCPP_WARN(this->get_logger(), "Motor command should have exactly 6 values");
    return;
  }

  float positions[6];
  for (int i = 0; i < 6; i++) {
    positions[i] = msg->data[i];
    target_positions_[i] = positions[i];
  }

  device_->setMotorPositions(positions, enumMovePattern_SmoothPursuit, enumMoveBase_Independent);
}

void BinosenseDriver::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float delta[6] = {0};

  delta[0] = msg->angular.y * 5.0f;
  delta[3] = msg->angular.y * 5.0f;

  delta[2] = -msg->angular.z * 5.0f;
  delta[5] = -msg->angular.z * 5.0f;

  delta[1] = msg->angular.x * 2.0f;
  delta[4] = msg->angular.x * 2.0f;

  for (int i = 0; i < 6; i++) {
    target_positions_[i] = current_positions_[i] + delta[i];
  }

  device_->setMotorRelativePositions(delta, enumMovePattern_SmoothPursuit, enumMoveBase_RightEye);
}

// ============================================================================
// Service callbacks
// ============================================================================

void BinosenseDriver::setMotorPositionSrv(
  const std::shared_ptr<binosense_ros2::srv::SetMotorPosition::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetMotorPosition::Response> res)
{
  BE_MovePatternType pattern = (req->move_pattern == 0) ? enumMovePattern_Saccade : enumMovePattern_SmoothPursuit;
  BE_MoveBaseType base;
  switch (req->move_base) {
    case 1: base = enumMoveBase_LeftEye; break;
    case 2: base = enumMoveBase_RightEye; break;
    default: base = enumMoveBase_Independent; break;
  }

  float angles[6];
  for (int i = 0; i < 6; i++) angles[i] = req->angles[i];

  bool ok;
  if (req->absolute) {
    ok = device_->setMotorPositions(angles, pattern, base);
  } else {
    ok = device_->setMotorRelativePositions(angles, pattern, base);
  }

  res->success = ok;
  res->message = ok ? "OK" : "Device not connected";
}

void BinosenseDriver::setMotorSpeedSrv(
  const std::shared_ptr<binosense_ros2::srv::SetMotorSpeed::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetMotorSpeed::Response> res)
{
  bool active[6];
  float speeds[6];
  for (int i = 0; i < 6; i++) {
    active[i] = req->active_flag[i];
    speeds[i] = req->speeds[i];
  }
  res->success = device_->setMotorSpeeds(speeds, active);
  res->message = res->success ? "OK" : "Device not connected";
}

void BinosenseDriver::setCameraExposureSrv(
  const std::shared_ptr<binosense_ros2::srv::SetCameraExposure::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetCameraExposure::Response> res)
{
  BinosenseDevice::EyeSide side;
  if (req->camera == 0) side = BinosenseDevice::RIGHT_EYE;
  else if (req->camera == 1) side = BinosenseDevice::LEFT_EYE;
  else side = BinosenseDevice::BOTH_EYES;

  res->success = device_->setExposure(side, req->auto_exposure, req->exposure_time);
  res->message = res->success ? "OK" : "Device not connected";
}

void BinosenseDriver::setCameraWhiteBalanceSrv(
  const std::shared_ptr<binosense_ros2::srv::SetCameraWhiteBalance::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetCameraWhiteBalance::Response> res)
{
  BinosenseDevice::EyeSide side;
  if (req->camera == 0) side = BinosenseDevice::RIGHT_EYE;
  else if (req->camera == 1) side = BinosenseDevice::LEFT_EYE;
  else side = BinosenseDevice::BOTH_EYES;

  res->success = device_->setWhiteBalance(side, req->auto_white_balance, req->temperature);
  res->message = res->success ? "OK" : "Device not connected";
}

void BinosenseDriver::setCameraGainSrv(
  const std::shared_ptr<binosense_ros2::srv::SetCameraGain::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetCameraGain::Response> res)
{
  BinosenseDevice::EyeSide side;
  if (req->camera == 0) side = BinosenseDevice::RIGHT_EYE;
  else if (req->camera == 1) side = BinosenseDevice::LEFT_EYE;
  else side = BinosenseDevice::BOTH_EYES;

  res->success = device_->setGain(side, req->auto_gain, req->gain);
  res->message = res->success ? "OK" : "Device not connected";
}

void BinosenseDriver::setVORSrv(
  const std::shared_ptr<binosense_ros2::srv::SetVOR::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetVOR::Response> res)
{
  device_->setVOR(req->vor_eye, req->vor_neck);
  res->success = true;
  res->message = "OK";
}

void BinosenseDriver::setSVSrv(
  const std::shared_ptr<binosense_ros2::srv::SetSV::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetSV::Response> res)
{
  device_->setSV(req->onoff);
  res->success = true;
  res->message = "OK";
}

void BinosenseDriver::setDepthControlSrv(
  const std::shared_ptr<binosense_ros2::srv::SetDepthControl::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetDepthControl::Response> res)
{
  res->success = device_->setDepthControl(req->enabled, req->sv_enabled, req->precision, req->distance_min, req->distance_max);
  res->message = res->success ? "OK" : "Device not connected";
}

void BinosenseDriver::saveDataSrv(
  const std::shared_ptr<binosense_ros2::srv::SaveData::Request> req,
  std::shared_ptr<binosense_ros2::srv::SaveData::Response> res)
{
  switch (req->action) {
    case 0:
      device_->saveBeData(true, req->folder_path);
      res->success = true;
      res->message = "Data save started";
      break;
    case 1:
      device_->saveBeData(false, req->folder_path);
      res->success = true;
      res->message = "Data save stopped";
      break;
    case 2:
      res->success = device_->snapBeData(req->description, req->folder_path);
      res->message = res->success ? "Data snapped" : "Snap failed";
      break;
    case 3: {
      BE_GeneralData data;
      if (device_->getBeDataRaw(data)) {
        res->success = device_->recordBeData(&data, req->description, req->folder_path);
        res->message = res->success ? "Data recorded" : "Record failed";
      } else {
        res->success = false;
        res->message = "No data available";
      }
      break;
    }
    case 4:
      device_->stopRecordBeData();
      res->success = true;
      res->message = "Recording stopped";
      break;
    default:
      res->success = false;
      res->message = "Unknown action";
  }
}

void BinosenseDriver::setImageResolutionSrv(
  const std::shared_ptr<binosense_ros2::srv::SetImageResolution::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetImageResolution::Response> res)
{
  if (req->transfer_only) {
    device_->setImageResolutionTransfer(req->width, req->height);
  } else {
    device_->setImageResolution(req->width, req->height);
  }
  res->success = true;
  res->message = "OK";
}

void BinosenseDriver::setExtraModuleSrv(
  const std::shared_ptr<binosense_ros2::srv::SetExtraModule::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetExtraModule::Response> res)
{
  if (req->module_type > 6) {
    res->success = false;
    res->message = "Invalid module type";
    return;
  }
  BE_ExtraModuleType type = static_cast<BE_ExtraModuleType>(req->module_type);
  device_->setExtraModuleFunction(type, req->param0, req->param1, req->param2);
  res->success = true;
  res->message = "OK";
}

void BinosenseDriver::getMotorLimitsSrv(
  const std::shared_ptr<binosense_ros2::srv::GetMotorLimits::Request> req,
  std::shared_ptr<binosense_ros2::srv::GetMotorLimits::Response> res)
{
  float up = 0, down = 0;

  if (req->motor_type >= 100) {
    // Neck motors
    BE_MotorType_Neck neck_type;
    switch (req->motor_type) {
      case 100: neck_type = enumNeckPitch; break;
      case 101: neck_type = enumNeckRoll; break;
      case 102: neck_type = enumNeckYaw; break;
      default: neck_type = enumNeckAllMotor; break;
    }
    res->success = device_->getNeckMotorLimits(neck_type, up, down);
  } else {
    BE_MotorType motor_type = static_cast<BE_MotorType>(req->motor_type);
    res->success = device_->getMotorLimits(motor_type, up, down);
  }

  res->up_limit = up;
  res->down_limit = down;
}

void BinosenseDriver::setNeckPositionSrv(
  const std::shared_ptr<binosense_ros2::srv::SetNeckPosition::Request> req,
  std::shared_ptr<binosense_ros2::srv::SetNeckPosition::Response> res)
{
  if (!device_->hasNeck()) {
    res->success = false;
    res->message = "No neck module connected";
    return;
  }

  bool active[3] = {true, true, true};
  float angles[3];
  for (int i = 0; i < 3; i++) angles[i] = req->angles[i];

  BE_MovePatternType pattern = (req->move_pattern == 0) ? enumMovePattern_Saccade : enumMovePattern_SmoothPursuit;

  if (req->absolute) {
    res->success = device_->setNeckAbsolutePosition(active, angles, pattern);
  } else {
    res->success = device_->setNeckRelativePosition(active, angles, pattern);
  }
  res->message = res->success ? "OK" : "Failed";
}

}  // namespace binosense_ros2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<binosense_ros2::BinosenseDriver>());
  rclcpp::shutdown();
  return 0;
}
