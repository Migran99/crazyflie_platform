/*!*******************************************************************************************
 *  \file       crazyflie_platform.hpp
 *  \brief      Header for the crazyflie_platform interface.
 *  \authors    Miguel Granero Ramos
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#ifndef CRAZYFLIE_PLATFORM_HPP_
#define CRAZYFLIE_PLATFORM_HPP_

#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Crazyflie.h>
#include <Eigen/Dense>

struct logBattery
{
    float pm_vbat;
    uint8_t charge_percent;
}__attribute__((packed));

class CrazyfliePlatform : public as2::AerialPlatform
{
public:
    CrazyfliePlatform();

    /*  --  AS2 FUNCTIONS --  */

    void configureSensors();

    bool ownSetArmingState(bool state);
    bool ownSetOffboardControl(bool offboard);
    bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg);
    bool ownSendCommand();

    /*  --  CRAZYFLIE FUNCTIONS --  */

    void listVariables();
    void listParams();
    void pingCB();
    void onLogIMU(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
    void onLogOdomOri(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
    void onLogOdomPos(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
    void onLogBattery(uint32_t /*time_in_ms*/, struct logBattery *data);
    void updateOdom();
    void externalOdomCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


    /*  --  AUX FUNCTIONS --  */

    Eigen::Vector3d quaternion2Euler(geometry_msgs::msg::Quaternion quat);

private:
    std::shared_ptr<Crazyflie> cf_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    bool is_connected_;
    bool is_armed_;
    std::string uri_;
    uint8_t controller_type_;
    uint8_t estimator_type_;

    /*  --  SENSORS --  */

    // Odometry
    std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_estimate_ptr_;
    double odom_buff_[10];
    std::function<void(uint32_t, std::vector<double> *, void *)> cb_odom_ori_;
    std::shared_ptr<LogBlockGeneric> odom_logBlock_ori_;
    bool ori_rec_;

    std::function<void(uint32_t, std::vector<double> *, void *)> cb_odom_pos_;
    std::shared_ptr<LogBlockGeneric> odom_logBlock_pos_;
    bool pos_rec_;

    // IMU
    std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
    double imu_buff_[6];
    std::function<void(uint32_t, std::vector<double> *, void *)> cb_imu_;
    std::shared_ptr<LogBlockGeneric> imu_logBlock_;

    // Battery
    std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
    unsigned char battery_buff_;
    
    std::unique_ptr<LogBlock<struct logBattery>> bat_logBlock_;
    std::function<void(uint32_t, struct logBattery*)> cb_bat_;

    // Optitrack
    bool external_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_odom_sub_;
    std::string external_odom_topic_;
    long last_external_update_;
    
};

#endif