#include "MvCameraControl.h"

// Standard C++ headers
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS headers
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera {

class HikCameraNode : public rclcpp::Node {
 public:
  explicit HikCameraNode(const rclcpp::NodeOptions& options)
      : Node("hik_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // -------------------------------------------------------------------------
    // 1. Device Enumeration (Search for Camera)
    // -------------------------------------------------------------------------
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret_code = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d",
                device_list.nDeviceNum);

    // If no camera is found, enter a loop to retry until a device is connected.
    // This prevents the node from crashing if the camera is plugged in late.
    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found! Retrying...");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", ret_code);
      // Sleep for 1 second to prevent high CPU usage during polling.
      std::this_thread::sleep_for(std::chrono::seconds(1));
      ret_code = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    // -------------------------------------------------------------------------
    // 2. Initialize Camera Handle
    // -------------------------------------------------------------------------
    // Create a handle for the first found device (index 0).
    ret_code = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (ret_code != MV_OK) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create handle: [%x]",
                   ret_code);
      return;
    }

    // Open the device.
    ret_code = MV_CC_OpenDevice(camera_handle_);
    if (ret_code != MV_OK) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open device: [%x]", ret_code);
      return;
    }

    // -------------------------------------------------------------------------
    // 3. Image Format Configuration
    // -------------------------------------------------------------------------
    // Get camera capabilities (max width/height) to prepare buffers.
    // We use the default resolution configured in the camera.
    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    // Initialize pixel conversion parameters.
    // We convert Raw Bayer data from the camera to RGB8 for ROS compatibility.
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // -------------------------------------------------------------------------
    // 4. ROS Publisher Setup
    // -------------------------------------------------------------------------
    // Use SensorData QoS (Best Effort) for low-latency video streaming.
    bool use_sensor_data_qos =
        this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                   : rmw_qos_profile_default;

    camera_pub_ =
        image_transport::create_camera_publisher(this, "image_raw", qos);

    // Initialize ROS parameters and sync them with hardware settings.
    DeclareParameters();

    // Start image acquisition from the camera.
    MV_CC_StartGrabbing(camera_handle_);

    // -------------------------------------------------------------------------
    // 5. Camera Calibration Info
    // -------------------------------------------------------------------------
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                                 camera_name_);

    // Load calibration file from the config directory.
    auto camera_info_url = this->declare_parameter(
        "camera_info_url", "package://hik_camera/config/camera_info.yaml");

    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                  camera_info_url.c_str());
    }

    // Register a callback for dynamic parameter reconfiguration.
    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::ParametersCallback, this,
                  std::placeholders::_1));

    // -------------------------------------------------------------------------
    // 6. Start Capture Thread
    // -------------------------------------------------------------------------
    // Use a separate thread for grabbing frames to avoid blocking the ROS
    // executor.
    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;  // Struct to hold the raw frame from SDK.
      int ret = MV_OK;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      // Set default frame_id if not provided by the calibration file.
      std::string frame_id = "camera_optical_frame";
      if (!camera_info_msg_.header.frame_id.empty()) {
        frame_id = camera_info_msg_.header.frame_id;
      }

      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        // Attempt to retrieve a frame with a 1000ms timeout.
        ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);

        // Record timestamp immediately after acquisition for higher accuracy.
        rclcpp::Time capture_time = this->now();

        if (MV_OK == ret) {
          // --- Data Conversion Logic ---

          // Calculate the required size for an RGB8 image (Width * Height * 3).
          size_t required_size =
              out_frame.stFrameInfo.nWidth * out_frame.stFrameInfo.nHeight * 3;

          // CRITICAL FIX: Resize the vector if the size doesn't match.
          // In the original code, reserve() was used, but size() remained 0,
          // causing the SDK conversion to fail.
          if (image_msg_.data.size() != required_size) {
            image_msg_.data.resize(required_size);
          }

          // Point the SDK conversion buffer to the ROS message data vector.
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          // Perform pixel conversion (Raw Bayer -> RGB8).
          int convert_ret =
              MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          if (convert_ret == MV_OK) {
            // Populate ROS message headers.
            image_msg_.header.stamp = capture_time;
            image_msg_.header.frame_id = frame_id;
            image_msg_.height = out_frame.stFrameInfo.nHeight;
            image_msg_.width = out_frame.stFrameInfo.nWidth;
            image_msg_.step = out_frame.stFrameInfo.nWidth * 3;

            // Sync CameraInfo timestamp with the image.
            camera_info_msg_.header = image_msg_.header;

            // Publish the synchronized Image and CameraInfo.
            camera_pub_.publish(image_msg_, camera_info_msg_);

            fail_count_ = 0;  // Reset failure counter on success.
          } else {
            RCLCPP_ERROR(this->get_logger(), "Pixel conversion failed: [%x]",
                         convert_ret);
          }

          // Release the internal SDK buffer (must be done even if conversion
          // fails).
          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

        } else {
          // --- Error Handling ---
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! Ret: [%x]", ret);

          // Attempt to restart grabbing to recover from errors.
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_count_++;
        }

        // If continuous failures occur, shutdown the node to prevent zombie
        // state.
        if (fail_count_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed continuously!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override {
    // Ensure the capture thread is stopped before releasing resources.
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

 private:
  /**
   * @brief Reads camera hardware capabilities and declares ROS parameters.
   * This ensures ROS parameters respect the camera's Min/Max limits.
   * Also prints a summary of loaded parameters.
   */
  void DeclareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;

    // --- 1. Exposure Time (Integer Parameter) ---
    // Read hardware limits for exposure time.
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.description = "Exposure time in microseconds";

    // Note: We cast to int64_t because ROS integer params are 64-bit.
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value =
        static_cast<int64_t>(f_value.fMin);
    param_desc.integer_range[0].to_value = static_cast<int64_t>(f_value.fMax);

    // Declare the parameter. If not found in YAML, use default (5000).
    int64_t exposure_time_param =
        this->declare_parameter("exposure_time", 5000, param_desc);

    // Write the initial value to the camera hardware.
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime",
                        static_cast<float>(exposure_time_param));

    // --- 2. Gain (Float/Double Parameter) ---
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.description = "Gain";

    // Clear integer constraints and set floating point constraints.
    param_desc.integer_range.clear();
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = f_value.fMin;
    float_range.to_value = f_value.fMax;
    float_range.step = 0.1;
    param_desc.floating_point_range = {float_range};

    // Declare parameter. Default to the current hardware value.
    double gain_param = this->declare_parameter(
        "gain", static_cast<double>(f_value.fCurValue), param_desc);

    // Write the initial value to the camera hardware.
    MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain_param));

    // --- FEATURE: Print all loaded parameters ---
    RCLCPP_INFO(this->get_logger(), "======================================");
    RCLCPP_INFO(this->get_logger(), "   HikCamera Parameter Summary        ");
    RCLCPP_INFO(this->get_logger(), "======================================");
    RCLCPP_INFO(this->get_logger(), " Exposure Time : %ld us", exposure_time_param);
    RCLCPP_INFO(this->get_logger(), " Gain          : %.2f", gain_param);
    RCLCPP_INFO(this->get_logger(), "======================================");
  }

  /**
   * @brief Callback function triggered when parameters are changed dynamically
   * (e.g., via command line or RQt).
   * Logs the change request.
   */
  rcl_interfaces::msg::SetParametersResult ParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
      if (param.get_name() == "exposure_time") {
        auto new_val = param.as_int();
        // --- FEATURE: Log parameter change ---
        RCLCPP_INFO(this->get_logger(),
                    "[Dynamic Reconfigure] Exposure Time -> %ld", new_val);

        // Handle Exposure Time change.
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime",
                                         static_cast<float>(new_val));
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " +
                          std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        auto new_val = param.as_double();
        // --- FEATURE: Log parameter change ---
        RCLCPP_INFO(this->get_logger(), "[Dynamic Reconfigure] Gain -> %.2f",
                    new_val);

        // Handle Gain change.
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain",
                                         static_cast<float>(new_val));
        if (MV_OK != status) {
          result.successful = false;
          result.reason =
              "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        // Handle unknown parameters if necessary.
      }
    }
    return result;
  }

  // --- Member Variables ---
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;

  void* camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)