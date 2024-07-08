/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <stdint.h>

#include <chrono>
#include <iostream>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
   public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        vehicle_control_mode_sub_ =
            this->create_subscription<VehicleControlMode>(
                "/fmu/out/vehicle_control_mode", rclcpp::SensorDataQoS(),
                [this](VehicleControlMode::UniquePtr msg)
                {
                    static bool is_init = false;
                    if (!is_init)
                    {
                        if (msg->flag_control_offboard_enabled)
                        {
                            RCLCPP_INFO(this->get_logger(),
                                        "Current mode is offboard, swithcing "
                                        "to position mode");
                            is_offboard_mode_ = true;
                        }
                        else
                        {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "Current mode is not offboard, swithcing to "
                                "offboard mode");
                            is_offboard_mode_ = false;
                        }
                        is_init = true;
                    }
                    else
                    {
                        return;
                    }
                });
        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void
        {
            if (offboard_setpoint_counter_ == 10)
            {
                // Change to Offboard mode after 10 setpoints
                // print only once?
                RCLCPP_INFO(
                    this->get_logger(),
                    "This is a Switch of  Offboard mode and Position mode");
                //                     param1 is set to 1 to enable the custom
                //                     mode.
                // param2 is set to 6 to indicate the offboard mode.
                // Other options for param2 include:
                // param2 = 1 (MANUAL)
                // param2 = 2 (ALTCTL)
                // param2 = 3 (POSCTL)
                // param2 = 4 (AUTO)
                // param2 = 5 (ACRO)
                // param2 = 6 (OFFBOARD)
                // param2 = 7 (STABILIZED)
                // param2 = 8 (RATTITUDE)
                if (!is_offboard_mode_)
                {
                    this->publish_vehicle_command(
                        VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    // Arm the vehicle
                    this->arm();
                }
                else
                {
                    this->publish_vehicle_command(
                        VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

                    // Disarm the vehicle
                }
            }

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11)
            {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();

   private:
    rclcpp::TimerBase::SharedPtr timer_;
    bool                         is_offboard_mode_{false};
    rclcpp::Publisher<OffboardControlMode>::SharedPtr
        offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr
        vehicle_control_mode_sub_;
    // std::atomic<uint64_t> timestamp_;  //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;  //!< counter for the number of
                                          //!< setpoints sent

    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD
 * codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
                                              float param2)
{
    VehicleCommand msg{};
    msg.param1           = param1;
    msg.param2           = param2;
    msg.command          = command;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}