#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

#include "gpg_remote/gpg_remote_hw.hpp"

#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("GPGRemoteHardware"), x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("GPGRemoteHardware"), x)

int GPGRemoteHardware::connect()
{
  int optval = 1;

  struct sockaddr_in addr;
  struct hostent *server;

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

  bzero((char *) &addr, sizeof(addr));
  addr.sin_family = AF_INET;
  
  server = gethostbyname(host_.c_str());
  if (!server)
  {
    ROS_ERROR_STREAM("Could not resolve GPG3 hostname '" << host_ << "'");
    return -1;
  }

  bcopy((char *)server->h_addr, (char *)&addr.sin_addr.s_addr, server->h_length);
  addr.sin_port = htons(port_);

  ROS_INFO_STREAM("Connecting to GPG3 at " << host_ << ":" << port_);
  while (::connect(fd,(struct sockaddr *) &addr,sizeof(addr)) < 0 && rclcpp::ok())
  {
    ROS_ERROR_STREAM("Could not connect to GPG3: " << strerror(errno));
    sleep(1);
  }

  return fd;
}

hardware_interface::CallbackReturn GPGRemoteHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
    
  host_ = info_.hardware_parameters["host"];
  port_ = std::stoul(info_.hardware_parameters["port"]);
  
  if (info_.joints.size() != 3)
  {
    ROS_ERROR_STREAM("Expected 3 joints, got " << info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Wheels  
  for (int ii=0; ii < 2; ++ii)
  {
    hardware_interface::ComponentInfo & joint = info_.joints[ii];
    
    if (joint.command_interfaces.size() != 1)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " has " << joint.command_interfaces.size() << " command interfaces, expected 1");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " has command interface " << joint.command_interfaces[0].name << ", expected " << hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " has " << joint.state_interfaces.size() << " state interfaces, expected 2");
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " state interface 0 is " << joint.state_interfaces[0].name << ", expected " << hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " state interface 1 is " << joint.state_interfaces[1].name << ", expected " << hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }
  
  // Servo
  hardware_interface::ComponentInfo & joint = info_.joints[2];
  if (joint.command_interfaces.size() != 1)
  {
    ROS_ERROR_STREAM("Joint " << joint.name << " has " << joint.command_interfaces.size() << " command interfaces, expected 1");
    return CallbackReturn::ERROR;
  }
  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    ROS_ERROR_STREAM("Joint " << joint.name << " has command interface " << joint.command_interfaces[0].name << ", expected " << hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }
  if (joint.state_interfaces.size() != 1)
  {
    ROS_ERROR_STREAM("Joint " << joint.name << " has " << joint.state_interfaces.size() << " state interfaces, expected 1");
    return CallbackReturn::ERROR;
  }
  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    ROS_ERROR_STREAM("Joint " << joint.name << " state interface 0 is " << joint.state_interfaces[0].name << ", expected " << hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }
  
  if (info_.sensors.size() != 2)
  {
    ROS_ERROR_STREAM("Expected 2 sensors, got " << info_.sensors.size());
    return CallbackReturn::ERROR;
  }
   
  // Line
  hardware_interface::ComponentInfo & line = info_.sensors[0];
  if (line.state_interfaces.size() != 5)
  {
    ROS_ERROR_STREAM("Sensor " << line.name << " has " << line.state_interfaces.size() << " state interfaces, expected 1");
    return CallbackReturn::ERROR;
  }
  
  for (size_t ii = 0; ii != 5; ++ii)
  {
    if (line.state_interfaces[ii].name != "level" + std::to_string(ii))
    {
      ROS_ERROR_STREAM("Sensor " << line.name << " state interface " << ii << " is " << line.state_interfaces[0].name << ", expected level" << std::to_string(ii));
      return CallbackReturn::ERROR;
    }
  }
  
  // Battery
  hardware_interface::ComponentInfo & battery = info_.sensors[1];
  if (battery.state_interfaces.size() != 1)
  {
    ROS_ERROR_STREAM("Sensor " << battery.name << " has " << battery.state_interfaces.size() << " state interfaces, expected 1");
    return CallbackReturn::ERROR;
  }
  if (battery.state_interfaces[0].name != "voltage")
  {
    ROS_ERROR_STREAM("Sensor " << battery.name << " state interface 0 is " << battery.state_interfaces[0].name << ", expected voltage");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GPGRemoteHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t ii = 0; ii != 3; ++ii)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[ii].name, hardware_interface::HW_IF_POSITION, &pos_[ii]
            )
        );
        
        if (ii < 2)
        {
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(
                  info_.joints[ii].name, hardware_interface::HW_IF_VELOCITY, &vel_[ii]
              )
          );
        }
    }
    
    for (size_t ii = 0; ii != 5; ++ii)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.sensors[0].name, "level" + std::to_string(ii), &line_[ii]
            )
        );
    }
    
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            info_.sensors[1].name, "voltage", &battery_
        )
    );
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GPGRemoteHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t ii = 0; ii != 3; ++ii) {
        if (ii < 2)
        {
          command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                  info_.joints[ii].name, hardware_interface::HW_IF_VELOCITY, &cmd_[ii]
              )
          );
        }
        else
        {
          command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                  info_.joints[ii].name, hardware_interface::HW_IF_POSITION, &cmd_[ii]
              )
          );
        }
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn GPGRemoteHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    for (size_t ii = 0; ii != 3; ++ii)
    {
      pos_[ii] = 0.0f;
      if (ii < 2) vel_[ii] = 0.0f;
      cmd_[ii] = 0.0f;
    }
    
    for (size_t ii = 0; ii != 3; ++ii)
      line_[ii] = 0.0f;
    battery_ = 0.0f;
    
    conn_ = connect();
    
    if (conn_ <= 0)
      return CallbackReturn::ERROR;

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GPGRemoteHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    close(conn_);

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GPGRemoteHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  int available=0;
  GPGRemoteStatusGrove msg;
  
  int err;
  socklen_t sz = sizeof(err);
  getsockopt(conn_, SOL_SOCKET, SO_ERROR, &err, &sz);
  
  if (err)
  {
    ROS_ERROR_STREAM("Socket error");
    close(conn_);
    conn_ = connect();
  }  

  if (conn_ < 0)
  {
    ROS_INFO_STREAM("NODE SHOULD EXIT");
    return hardware_interface::return_type::ERROR; // exit(1)
  }

  ioctl(conn_, FIONREAD, &available);
  
  while (available >= 4)
  {
    if (::read(conn_, &msg, 4) < 4)
    {
      ROS_ERROR_STREAM("Could not read GPG3 status message header");
      close(conn_);
      return hardware_interface::return_type::ERROR;
    }
    
    if (::read(conn_, &((unsigned char*)&msg)[4], msg.size) < msg.size)
    {
      ROS_ERROR_STREAM("Could not read GPG3 status message");
      close(conn_);
      return hardware_interface::return_type::ERROR;
    }
  
    if (msg.size >= sizeof(GPGRemoteStatus)-4)
    {
      // Wheels
      for (int ii=0; ii<2; ++ii)
      {
        if (first_)
          pos_[ii] = msg.pos[ii]/MOTOR_TICKS_PER_RADIAN;
      
        double prevpos = pos_[ii];
        pos_[ii] = msg.pos[ii]/MOTOR_TICKS_PER_RADIAN;
        vel_[ii] = (pos_[ii]-prevpos)/period.seconds();
      }
      
      // Servo
      pos_[2] = cmd_[2];
      
      // Line sensor
      for (int ii=0; ii<5; ++ii)
        if (msg.line[ii] < 1024)
          line_[ii] = (double)msg.line[ii];
        
      // Battery voltage
      battery_ = msg.battery;
    }
    else
    {
      ROS_ERROR_STREAM("Invalid GPG3 status message size: " << msg.size);
      close(conn_);
      return hardware_interface::return_type::ERROR;
    }
      
    if (msg.size >= sizeof(GPGRemoteStatusGrove)-4)
    {
      // Distance sensors
      for (int ii=0; ii<4; ++ii)
      {
        double v = msg.distance[ii]/1024.*5;
        distance_[ii] = 0.07 + (16.2537 * pow(v, 4) - 129.893 * pow(v, 3) + 382.268 * pow(v, 2) - 512.611 * v + 306.439) / 100;
      }
      
      // Light sensors
      for (int ii=0; ii<2; ++ii)
        light_[ii] = msg.light[ii];
    }
    else
    {
      for (int ii=0; ii<4; ++ii)
        distance_[ii] = 0;
      for (int ii=0; ii<2; ++ii)
        light_[ii] = 0;
    }

    first_ = false;

    available = 0;
    ioctl(conn_, FIONREAD, &available);
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GPGRemoteHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  GPGRemoteCommand msg;

  msg.size = sizeof(GPGRemoteCommand)-4;

  msg.vel[0] = cmd_[0]*MOTOR_TICKS_PER_RADIAN;
  msg.vel[1] = cmd_[1]*MOTOR_TICKS_PER_RADIAN;
  msg.servo  = SERVO_CENTER_US + cmd_[2]*SERVO_US_PER_RADIAN;
  msg.led[0] = led_[0]*255;
  msg.led[1] = led_[1]*255;
  msg.led[2] = led_[2]*255;
  
  if (::write(conn_, &msg, sizeof(GPGRemoteCommand)) < sizeof(GPGRemoteCommand))
  {
    ROS_ERROR_STREAM("Could not write GPG3 command message: " << strerror(errno));
    close(conn_);
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(GPGRemoteHardware, hardware_interface::SystemInterface)
