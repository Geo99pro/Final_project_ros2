#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>
#include <string>

// From https://github.com/DexterInd/GoPiGo3/blob/master/Software/Python/gopigo3.py
#define MOTOR_GEAR_RATIO           120
#define ENCODER_TICKS_PER_ROTATION   6
#define MOTOR_TICKS_PER_RADIAN     ((MOTOR_GEAR_RATIO*ENCODER_TICKS_PER_ROTATION)/(2*M_PI))

#define SERVO_CENTER_US            1500
#define SERVO_RANGE_US             1850
#define SERVO_US_PER_RADIAN        (SERVO_RANGE_US/M_PI)

struct GPGRemoteStatus
{
  uint32_t size;
  int32_t pos[2];
  uint32_t line[5];
  float battery;
}; 

struct GPGRemoteStatusGrove
{
  uint32_t size;
  int32_t pos[2];
  uint32_t line[5];
  float battery;
  uint32_t distance[4];
  uint32_t light[2];
}; 

struct GPGRemoteCommand
{
  uint32_t size;
  int32_t vel[2];
  uint32_t servo;
  int8_t led[3];
} __attribute__((packed));

class GPGRemoteHardware : public hardware_interface::SystemInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(GPGRemoteHardware)

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    double cmd_[3];
    double pos_[3];
    double vel_[2];
    double led_[3];
    double line_[5];
    double battery_;
    double distance_[4];
    double light_[2];
    
    int conn_;
    bool first_;
    
    std::string host_;
    int port_;
    
  public:
    GPGRemoteHardware() : conn_(-1), first_(true) { }

  private:
    int connect();
};
