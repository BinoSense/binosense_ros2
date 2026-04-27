#include <rclcpp/rclcpp.hpp>
#include <binosense_ros2/msg/binosense_data.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

class BinosenseDataRecorder : public rclcpp::Node
{
public:
  BinosenseDataRecorder()
  : Node("binosense_data_recorder"),
    frame_count_(0),
    is_recording_(false)
  {
    this->declare_parameter<std::string>("output_dir", ".");
    this->declare_parameter<bool>("auto_start", true);
    this->declare_parameter<int>("max_frames", -1);  // -1 for unlimited

    this->get_parameter("output_dir", output_dir_);
    this->get_parameter("auto_start", auto_start_);
    this->get_parameter("max_frames", max_frames_);

    // Create output directory if it doesn't exist
    std::string cmd = "mkdir -p " + output_dir_;
    system(cmd.c_str());

    // Create CSV file for metadata
    std::string csv_path = output_dir_ + "/data.csv";
    csv_file_.open(csv_path);
    writeCSVHeader();

    subscription_ = this->create_subscription<binosense_ros2::msg::BinosenseData>(
      "binosense_data", 10,
      std::bind(&BinosenseDataRecorder::dataCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Binosense Data Recorder Started");
    RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());

    if (auto_start_) {
      is_recording_ = true;
      RCLCPP_INFO(this->get_logger(), "Auto-start recording...");
    }

    printHelp();
  }

  ~BinosenseDataRecorder()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  void printHelp()
  {
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "=== Binosense Data Recorder ===");
    RCLCPP_INFO(this->get_logger(), "Records synchronized Binosense data to files");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  - output_dir: Output directory (default: .)");
    RCLCPP_INFO(this->get_logger(), "  - auto_start: Start recording immediately (default: true)");
    RCLCPP_INFO(this->get_logger(), "  - max_frames: Max frames to record, -1 for unlimited (default: -1)");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Output files:");
    RCLCPP_INFO(this->get_logger(), "  - data.csv: Metadata CSV with all frame information");
    RCLCPP_INFO(this->get_logger(), "  - left_XXXXXX.png: Left eye images");
    RCLCPP_INFO(this->get_logger(), "  - right_XXXXXX.png: Right eye images");
    RCLCPP_INFO(this->get_logger(), "  - both_XXXXXX.png: Both eyes (side-by-side) images");
    RCLCPP_INFO(this->get_logger(), " ");
  }

  void writeCSVHeader()
  {
    csv_file_ << "frame_number,ros_time,device_frame_id,device_timestamp,";
    csv_file_ << "left_image,right_image,both_image,";
    csv_file_ << "motor_r_pitch,motor_r_roll,motor_r_yaw,";
    csv_file_ << "motor_l_pitch,motor_l_roll,motor_l_yaw,";
    csv_file_ << "gps_longitude,gps_latitude,gps_height,gps_time,";
    csv_file_ << "is_moving_fastly";

    // Add IMU columns (16 sensors x 4 values = 64 elements)
    for (int i = 0; i < 16; i++) {
      for (int j = 0; j < 4; j++) {
        csv_file_ << ",imu_" << i << "_" << j;
      }
    }

    csv_file_ << std::endl;
  }

  void saveImage(const sensor_msgs::msg::Image & img_msg, const std::string & prefix)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr;
      if (img_msg.encoding == "rgb8" || img_msg.encoding == "bgr8") {
        cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg.encoding);
      } else {
        cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
      }

      std::stringstream ss;
      ss << output_dir_ << "/" << prefix << "_"
         << std::setw(6) << std::setfill('0') << frame_count_ << ".png";

      cv::imwrite(ss.str(), cv_ptr->image);

      last_image_files_[prefix] = ss.str();
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
      last_image_files_[prefix] = "";
    }
  }

  void dataCallback(const binosense_ros2::msg::BinosenseData::SharedPtr msg)
  {
    if (!is_recording_) {
      return;
    }

    if (max_frames_ > 0 && frame_count_ >= max_frames_) {
      RCLCPP_INFO(this->get_logger(), "Reached max frames (%d), stopping recording", max_frames_);
      is_recording_ = false;
      return;
    }

    frame_count_++;

    // Save images
    last_image_files_.clear();

    if (msg->left_image.width > 0 && msg->left_image.height > 0) {
      saveImage(msg->left_image, "left");
    }

    if (msg->right_image.width > 0 && msg->right_image.height > 0) {
      saveImage(msg->right_image, "right");
    }

    if (msg->both_image.width > 0 && msg->both_image.height > 0) {
      saveImage(msg->both_image, "both");
    }

    // Write to CSV
    csv_file_ << frame_count_ << ",";
    csv_file_ << msg->header.stamp.sec << "." << std::setw(9) << std::setfill('0') << msg->header.stamp.nanosec << ",";
    csv_file_ << msg->frame_id << ",";
    csv_file_ << msg->timestamp << ",";

    csv_file_ << (last_image_files_.count("left") ? last_image_files_["left"] : "") << ",";
    csv_file_ << (last_image_files_.count("right") ? last_image_files_["right"] : "") << ",";
    csv_file_ << (last_image_files_.count("both") ? last_image_files_["both"] : "") << ",";

    // Motor data (6 motors)
    for (int i = 0; i < 6; i++) {
      csv_file_ << msg->motor_data[i];
      if (i < 5) csv_file_ << ",";
    }
    csv_file_ << ",";

    // GPS data
    csv_file_ << msg->gps_data[0] << "," << msg->gps_data[1] << "," << msg->gps_data[2] << ",";
    csv_file_ << msg->gps_time << ",";

    // Motion flag
    csv_file_ << msg->is_moving_fastly;

    // IMU data (16 sensors x 4 values, flattened to 64 elements)
    for (int i = 0; i < 16; i++) {
      for (int j = 0; j < 4; j++) {
        csv_file_ << "," << msg->imu_data[i * 4 + j];
      }
    }

    csv_file_ << std::endl;
    csv_file_.flush();

    if (frame_count_ % 25 == 0) {
      RCLCPP_INFO(this->get_logger(), "Recorded %d frames", frame_count_);
    }
  }

  std::string output_dir_;
  bool auto_start_;
  int max_frames_;

  int frame_count_;
  bool is_recording_;

  std::ofstream csv_file_;
  std::map<std::string, std::string> last_image_files_;

  rclcpp::Subscription<binosense_ros2::msg::BinosenseData>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BinosenseDataRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
