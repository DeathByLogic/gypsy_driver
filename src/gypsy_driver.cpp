/*
 * gypsydriver.cpp
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>

#include "beagleIO.h"
#include "sabertooth.h"
#include "gypsydriver.h"
#include "sensors.h"

// Global constants
double rate = 10.0;
Posistion current_pos;

// Global variables
Sabertooth ST(HBRIDGE_ADDRESS, &HBRIDGE_SERIAL_TX_PIN);

ros::Time current_time;
ros::Time last_time;

void command_callback( const geometry_msgs::Twist& cmd_msg) {
  ST.drive(cmd_msg.linear.x);
  ST.turn(cmd_msg.angular.z);
}

void motor_init() {
  // Init Serial port for H-Bridge
  HBRIDGE_SERIAL_TX_PIN.openPort(BR_38400, PB_NONE);

  // Init H-Bridge shutdown pin
  HBRIDGE_SHUTDOWN.exportPin();
  HBRIDGE_SHUTDOWN.setDirection(OUTPUT_PIN);
  HBRIDGE_SHUTDOWN.writePin(true);
  
  ST.setDeadband(HBRIDGE_DEADBAND);		// Set deadband width
  ST.setMinVoltage(HBRIDGE_MIN_VOLTAGE);	// Set battery cutoff
  ST.setTimeout(HBRIDGE_TIMEOUT);		// Set serial timeout
}

int main (int argc, char** argv) {
  motor_init();

  ros::init(argc, argv, "gypsy_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 50, command_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  //tf::TransformBroadcaster odom_broadcaster;

  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;

  long l_tick = 0;
  long r_tick = 0;

  long l_dt = 0;
  long r_dt = 0;

  ros::Rate r(rate);
  while (n.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();

    //calculate change in time (dt)
    double dtt = (current_time - last_time).toSec();
    
    // Read the number of tick counts and time from left sensor
    read_sensor(&SENSOR_I2C, LEFT_SENSOR_ADR, 0x21, sizeof(l_dt), &l_dt);
    read_sensor(&SENSOR_I2C, LEFT_SENSOR_ADR, 0x25, sizeof(l_tick), &l_tick);

    // Read the number of tick counts and time from left sensor
    read_sensor(&SENSOR_I2C, LEFT_SENSOR_ADR, 0x21, sizeof(r_dt), &r_dt);
    read_sensor(&SENSOR_I2C, LEFT_SENSOR_ADR, 0x25, sizeof(r_tick), &r_tick);

    update_location(&current_pos, l_tick, r_tick);

    r.sleep();
  }

  return 0;
}



