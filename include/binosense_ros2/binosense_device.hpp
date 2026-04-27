#ifndef BINOSENSE_ROS2__BINOSENSE_DEVICE_HPP_
#define BINOSENSE_ROS2__BINOSENSE_DEVICE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <mutex>

#include "bionic_eyes_c.h"
#include "bionic_eyes_cpp_wrapper.h"

namespace binosense_ros2
{

struct MotorState
{
  float position[6];
  float speed[6];
  bool enabled[6];
};

struct CameraState
{
  float exposure_time[2];
  float white_balance[2];
  int gain[2];
};

struct DepthData
{
  int id;
  std::vector<uint16_t> Z;
  std::vector<float> L0LnRT;
  std::vector<float> KLrect;
  BE_Image bgr_left;
};

class BinosenseDevice
{
public:
  enum MotorIndex
  {
    RIGHT_PITCH = 0,
    RIGHT_ROLL = 1,
    RIGHT_YAW = 2,
    LEFT_PITCH = 3,
    LEFT_ROLL = 4,
    LEFT_YAW = 5
  };

  enum EyeSide
  {
    RIGHT_EYE = 0,
    LEFT_EYE = 1,
    BOTH_EYES = 2
  };

  BinosenseDevice();
  ~BinosenseDevice();

  bool connect(const std::string & mode = "ic", const std::string & ip = "");
  void disconnect();
  bool isConnected() const;

  bool isDataReady();
  bool getImageData(EyeSide side, BE_Image & image);
  bool getStereoImages(BE_Image & left, BE_Image & right);
  bool getMotorState(MotorState & state);
  bool getCameraState(CameraState & state);

  bool setMotorPositions(const float positions[6],
                         BE_MovePatternType pattern = enumMovePattern_SmoothPursuit,
                         BE_MoveBaseType base = enumMoveBase_Independent);

  bool setMotorRelativePositions(const float positions[6],
                                  BE_MovePatternType pattern = enumMovePattern_SmoothPursuit,
                                  BE_MoveBaseType base = enumMoveBase_Independent);

  bool setMotorSpeeds(const float speeds[6], const bool active[6]);

  bool goHome();
  bool setCurrentAsHome();

  bool setExposure(EyeSide side, bool auto_exposure, float exposure_time);
  bool setWhiteBalance(EyeSide side, bool auto_wb, float temperature);
  bool setGain(EyeSide side, bool auto_gain, int gain);

  // VOR / SV
  void setVOR(bool vor_eye, bool vor_neck);
  void setSV(bool onoff);

  // Depth
  bool setDepthControl(bool onOff, bool svOnOff, int precision, float min, float max);
  bool getDepthInfo(DepthData & depth);
  bool getNewestDepthInfo(DepthData & depth);

  // Motor limits
  bool getMotorLimits(BE_MotorType type, float & up, float & down);
  bool getNeckMotorLimits(BE_MotorType_Neck type, float & up, float & down);

  // Neck control
  bool hasNeck();
  bool setNeckAbsolutePosition(const bool activeFlag[3], const float angle[3], BE_MovePatternType moveType);
  bool setNeckRelativePosition(const bool activeFlag[3], const float angle[3], BE_MovePatternType moveType);
  bool setUnionAbsolutePositionNeckEye(const bool activeFlag[3],
                                        const float eyeAngle[6], const float neckAngle[3],
                                        BE_MovePatternType moveType);
  bool setUnionRelativePositionNeckEye(const bool activeFlag[3],
                                        const float eyeAngle[6], const float neckAngle[3],
                                        BE_MovePatternType moveType);
  void goNeckInitPosition(BE_MotorType_Neck type = enumNeckAllMotor);

  // Image settings
  void setImageResolution(int width, int height);
  void setImageResolutionTransfer(int width, int height);
  void setImageColor(BE_ImageColorType type);
  void setImageColorTransfer(BE_ImageColorType type);
  void setDataRateTransfer(float rate);
  void setImageCompressionQuality(int quality);
  BE_Size getOriginImageResolution();
  float getMaxImageFrameRate();

  // Data rate
  void setBeDataRate(int rate);
  void setSyncCameraDelayTime(int timeDelay);
  void triggerDataTransmission();

  // Save / record / snap
  void saveBeData(bool startStop, const std::string & folderPath);
  bool snapBeData(const std::string & description, const std::string & folderPath);
  void stopSnapBeData();
  bool recordBeData(BE_GeneralData * data, const std::string & description, const std::string & folderPath);
  void stopRecordBeData();

  // Extra module
  void setExtraModuleFunction(BE_ExtraModuleType type, int para0, int para1, int para2);
  void setZoomFocalLength(uint8_t focalLength);
  void setWipers(uint8_t value);

  // Motor position speed
  void setMotorPositionSpeed(int saccadeSpeed, int pursuitSpeed);

  // Sync signal
  void setSyncSignalSource(bool flag);

  // Camera on/off
  void onOffCameraImage(bool flag[], bool onlyNetworkType);

  // Device info
  uint32_t getDeviceIp();
  const char* getDeviceIpStr();
  BE_Device_Type getDeviceType();

  uint64_t getFrameId() const { return current_frame_id_; }
  uint64_t getTimestamp() const { return current_timestamp_; }

  bool getBeDataRaw(BE_GeneralData & data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!connected_ || !device_) {
      return false;
    }
    device_->getBeData(data);
    current_frame_id_ = data.frame_id;
    current_timestamp_ = data.timestamp;
    return true;
  }

  bionic_eyes::BionicEyesWrapper * rawDevice() { return device_.get(); }

private:
  std::unique_ptr<bionic_eyes::BionicEyesWrapper> device_;
  BE_GeneralData current_data_;
  uint64_t current_frame_id_;
  uint64_t current_timestamp_;
  bool connected_;
  mutable std::mutex data_mutex_;
  std::string ip_str_buffer_;  // For storing IP string
};

}  // namespace binosense_ros2

#endif  // BINOSENSE_ROS2__BINOSENSE_DEVICE_HPP_
