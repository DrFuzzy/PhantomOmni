/*
 * ROS 2 package for SensAble PHANTOM Omni system with FireWire interface.
 *
 * Based on original code from Healthcare Robotics Lab at Georgia Tech.
 *
 * Copyright (c) 2022-2024 Dr. Kyriakos Deliparaschos
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "tf2/LinearMath/Vector3.h"
// #include "tf2/LinearMath/Transform.h"
// #include "tf2/LinearMath/Matrix3x3.h"

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduVector.h>
#include <HL/hl.h>
#include <pthread.h>

#include "phantom_omni/msg/omni_button_event.hpp"
#include "phantom_omni/msg/omni_feedback.hpp"

struct OmniState {
  hduVector3Dd position; // 3x1 vector of position
  hduVector3Dd velocity; // 3x1 vector of velocity
  hduVector3Dd
      inp_vel1; // 3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; // 3x1 history of position used for 2nd order
                          // backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduVector3Dd rot;
  hduVector3Dd joints;
  hduVector3Dd joint_torques;
  hduVector3Dd force; // 3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  hduVector3Dd lock_pos;
};

/**
 * @class PhantomROS2
 * @brief Defines ROS 2 Omni node
 * 
 * This class inherits from rclcpp::Node
 */
class PhantomROS2 : public rclcpp::Node {
public:
  PhantomROS2() : Node("omni_ros2"), count_(0) {
    // RCLCPP_INFO(this->get_logger(), "Creating PhantomROS2");
    OmniState *state;

    // Create publishers
    std::ostringstream stream00;
    stream00 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream00.str());
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        pose_topic_name.c_str(),
        100); // Publish on NAME/pose

    // joint_publisher =
    // this->create_publisher<sensor_msgs::msg::JointState>("joint", 100);

    std::ostringstream stream0;
    stream0 << omni_name << "/button";
    std::string button_topic = std::string(stream0.str());
    button_publisher =
        this->create_publisher<phantom_omni_ros2::msg::OmniButtonEvent>(
            button_topic.c_str(), 100); // Publish on NAME/button

    // Create subscribers
    std::ostringstream stream01;
    stream01 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream01.str());
    haptic_subscriber = this->create_subscription<std_msgs::msg::String>(
        force_feedback_topic.c_str(), 10,
        std::bind(&PhantomROS2::force_callback, this,
                  _1)); // Subscribe to NAME/force_feedback

    // Frame of force feedback (NAME_sensable)
    std::ostringstream stream2;
    stream2 << omni_name << "_sensable";
    sensable_frame_name = std::string(stream2.str());

    for (int i = 0; i < 7; i++) {
      std::ostringstream stream1;
      stream1 << omni_name << "_link" << i;
      link_names[i] = std::string(stream1.str());
    }

    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  // 3x1 history of velocity
    state->inp_vel2 = zeros;  // 3x1 history of velocity
    state->inp_vel3 = zeros;  // 3x1 history of velocity
    state->out_vel1 = zeros;  // 3x1 history of velocity
    state->out_vel2 = zeros;  // 3x1 history of velocity
    state->out_vel3 = zeros;  // 3x1 history of velocity
    state->pos_hist1 = zeros; // 3x1 history of position
    state->pos_hist2 = zeros; // 3x1 history of position
    state->lock = true;
    state->lock_pos = zeros;
  }

  void force_callback(const geometry_msgs::msg::Vector3 &omnifeed_forces);
  void publish_omni_state();

private:
  OmniState *state;

  // Declaration of publisher attributes
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<phantom_omni_ros2::msg::OmniButtonEvent>::SharedPtr
      button_publisher;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
  // omni_angles_publisher;

  // Declaration of subscriber attributes
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr haptic_subscriber;

  std::string omni_name;
  std::string sensable_frame_name;
  std::string link_names[7];

  // Declaration of the count_ attribute
  int count_;
};

/**
 * @brief Implement force feedback callback
 * @return void
 */
void PhantomROS2::force_callback(
    const geometry_msgs::msg::Vector3 &omnifeed_forces) {
  // Some people might not like this extra damping, but it
  // helps to stabilize the overall force feedback. It isn't
  // like we are getting direct impedance matching from the
  // omni anyway
  state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
  state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
  state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

  state->lock_pos[0] = omnifeed->position.x;
  state->lock_pos[1] = omnifeed->position.y;
  state->lock_pos[2] = omnifeed->position.z;
}

/**
 * @brief Implements state callback
 * @return void
 */
void PhantomROS2::publish_omni_state() {
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "waist";
  joint_state.position[0] = -state->thetas[1];
  joint_state.name[1] = "shoulder";
  joint_state.position[1] = state->thetas[2];
  joint_state.name[2] = "elbow";
  joint_state.position[2] = state->thetas[3];
  joint_state.name[3] = "yaw";
  joint_state.position[3] = -state->thetas[4] + M_PI;
  joint_state.name[4] = "pitch";
  joint_state.position[4] = -state->thetas[5] - 3 * M_PI / 4;
  joint_state.name[5] = "roll";
  joint_state.position[5] = -state->thetas[6] - M_PI;
  joint_pub.publish(joint_state);

  // Sample 'end effector' pose
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = link_names[6].c_str();
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose.position.x = 0.0; // was 0.03 to end of phantom
  pose_stamped.pose.orientation.w = 1.;
  pose_publisher.publish(pose_stamped);

  if ((state->buttons[0] != state->buttons_prev[0]) or
      (state->buttons[1] != state->buttons_prev[1])) {
    if ((state->buttons[0] == state->buttons[1]) and (state->buttons[0] == 1)) {
      state->lock = !(state->lock);
    }
    omni_msgs::OmniButtonEvent button_event;
    button_event.grey_button = state->buttons[0];
    button_event.white_button = state->buttons[1];
    state->buttons_prev[0] = state->buttons[0];
    state->buttons_prev[1] = state->buttons[1];
    button_publisher.publish(button_event);
  }
}

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
  OmniState *omni_state = static_cast<OmniState *>(pUserData);

  hdBeginFrame(hdGetCurrentDevice());

  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
  hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
  hdGetDoublev(HD_CURRENT_TRANSFORM, omni_state->pose_transform_matrix);

  // Position to metres
  for (unsigned int i = 0; i < 3; i++) {
    omni_state->position[i] /= 1000;
    omni_state->pose_transform_matrix[i + 12] /= 1000;
  }

  // Decouple joint 3
  omni_state->joints[2] = omni_state->joints[2] - omni_state->joints[1];

  // Workspace dimensions
  if (!omni_state->workspace_data_set) {
    HDdouble hddata[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, hddata);
    for (unsigned int i = 0; i < 6; i++)
      omni_state->max_workspace[i] = hddata[i] / 1000; // Set data in metres

    hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, hddata);
    for (unsigned int i = 0; i < 6; i++)
      omni_state->usable_workspace[i] = hddata[i] / 1000; // Set data in metres

    omni_state->workspace_data_set = true;
  }

  // Historial & velocity
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1 +
              omni_state->pos_hist2) /
             0.002; // mm/s, 2nd order backward dif
  omni_state->velocity =
      (.2196 * (vel_buff + omni_state->inp_vel3) +
       .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) /
          1000.0 -
      (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2 -
       0.7776 * omni_state->out_vel3); // cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  // Buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  // Lock
  if (omni_state->lock == true) {
    omni_state->force =
        1000 * (0.08 * (omni_state->lock_pos - omni_state->position) -
                0.001 * omni_state->velocity); // Magnitudes in mm and mm/s

    hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

    // printf("joint forces: %3.2f %3.2f %3.2f\n", omni_state->force[0],
    // omni_state->force[1], omni_state->force[2]);
  }
  // Set force/torque
  else {
    double f_norm = fabs(omni_state->force[0]) + fabs(omni_state->force[1]) +
                    fabs(omni_state->force[2]);
    double t_norm = fabs(omni_state->joint_torques[0]) +
                    fabs(omni_state->joint_torques[1]) +
                    fabs(omni_state->joint_torques[2]);

    /* printf("joint torques / forces: %3.2f %3.2f %3.2f   -   %3.2f %3.2f
       %3.2f
       -   %f %f\n", omni_state->joint_torques[0],
       omni_state->joint_torques[1], omni_state->joint_torques[2],
       omni_state->force[0], omni_state->force[1], omni_state->force[2],
       t_norm, f_norm); */

    double epsilon = 0.0001;

    if ((f_norm > epsilon) && (t_norm < epsilon)) {
      hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);
    } else if ((t_norm > epsilon) && (f_norm < epsilon)) {
      hdSetDoublev(HD_CURRENT_JOINT_TORQUE, omni_state->joint_torques);
    } else {
      hduVector3Dd joint_torques_zero, force_zero;
      for (unsigned int i = 0; i < 3; i++) {
        joint_torques_zero[i] = 0.0;
        force_zero[i] = 0.0;
      }

      hdSetDoublev(HD_CURRENT_FORCE, force_zero);
      hdSetDoublev(HD_CURRENT_JOINT_TORQUE, joint_torques_zero);
    }
  }

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback\n");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  // Plot values
  //     printf("angles:  %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f\n",
  //     omni_state->joints[0]*(180/M_PI), omni_state->joints[1]*(180/M_PI),
  //     (omni_state->joints[2])*(180/M_PI), omni_state->rot[0]*(180/M_PI),
  //     omni_state->rot[1]*(180/M_PI), omni_state->rot[2]*(180/M_PI));
  //
  //     printf("position x, y, z: %f %f %f \n", omni_state->position[0],
  //     omni_state->position[1], omni_state->position[2]); printf("velocity x,
  //     y, z, time: %f %f %f \n", omni_state->velocity[0],
  //     omni_state->velocity[1],omni_state->velocity[2]); std::cout << "
  //     --------------------------------- " << std::setw(4) << std::endl; for
  //     (unsigned int i=0; i<4; i++){
  //       for (unsigned int j=0; j<4; j++)
  //       std::cout << std::setw(5) << std::fixed << std::setprecision(2) <<
  //       omni_state->pose_transform_matrix[i+4*j] << "  "; std::cout <<
  //       std::endl;
  //     }
  //     std::cout << std::endl;
  //     printf("lock position x, y, z: %f %f %f \n", omni_state->lock_pos[0],
  //     omni_state->lock_pos[1], omni_state->lock_pos[2]);

  return HD_CALLBACK_CONTINUE;
}

/**
 * @brief Calibrates Phantom Device automatically - No character inputs
 * @return void
 */
void HHD_Auto_Calibration() {
  int calibrationStyle;
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    RCLCPP_INFO(node->get_logger(), "HD_CALIBRATION_ENCODER_RESET\n\n");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    RCLCPP_INFO(node->get_logger(), "HD_CALIBRATION_INKWELL\n\n");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    RCLCPP_INFO(node->get_logger(), "HD_CALIBRATION_AUTO\n\n");
  }

  do {
    hdUpdateCalibration(calibrationStyle);
    RCLCPP_INFO(node->get_logger(), "Calibrating...(put stylus in well)\n");
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      hduPrintError(stderr, &error, "Reset encoders failed!");
      break;
    }
  } while (hdCheckCalibration() != HD_CALIBRATION_OK);

  RCLCPP_INFO(node->get_logger(), "\n\nCalibration complete.\n");
}

/**
 * @brief Executes callbacks simultaneously using multithreading
 * @return void
 */
void *ros_publish(void *ptr) {
  PhantomROS2 *omni_ros2 = (PhantomROS2 *)ptr;
  int publish_rate;

  // omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
  // ros::Rate loop_rate(publish_rate);
  // ros::AsyncSpinner spinner(2);
  // spinner.start();

  // while (ros::ok()) {
  //     omni_ros->publish_omni_state();
  //     ros::spinOnce();
  //     loop_rate.sleep();
  // }

  rclcpp::Rate loop_rate(publish_rate);
  using rclcpp::executors::multi_threaded_executor::MultiThreadedExecutor;
  MultiThreadedExecutor executor;
  executor.add_node(omni_ros2);
  std::thread executor_thread(
      std::bind(&MultiThreadedExecutor::spin, &executor));

  while (rclcpp::ok()) {
    omni_ros2->publish_omni_state();
    // Start processing data from the node, the callbacks and the timer
    rclcpp::spin_some(omni_ros2);
    // rclcpp::spin(std::make_shared<PhantomROS2>());
    loop_rate.sleep();
  }

  return NULL;
}

/**
 * @brief Creates ROS 2 node
 */
int main(int argc, char *argv[]) {
  // Initialise Phantom
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    // hduPrintError(stderr, &error, "Failed to initialize haptic device");
    RCLCPP_INFO(node->get_logger(), "Failed to initialize haptic device %s",
                &error);
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "Found %s.\n\n",
              hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_INFO(node->get_logger(), "Failed to start the scheduler %s", &error);
    return -1;
  }
  HHD_Auto_Calibration();

  // Initialise ROS 2
  rclcpp::init(argc, argv);
  OmniState state;
  PhantomROS2 omni_ros2;
  omni_ros2.init(&state);
  hdScheduleAsynchronous(omni_state_callback, &state,
                         HD_MAX_SCHEDULER_PRIORITY);

  // Loop and publish
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void *)&omni_ros2);
  pthread_join(publish_thread, NULL);

  RCLCPP_INFO(node->get_logger(), "Ending Session....\n");
  hdStopScheduler();
  hdDisableDevice(hHD);

  // Shutdown the node when done
  rclcpp::shutdown();
  return 0;
}

// https://sir.upc.edu/wikis/roblab/index.php/Development/PhantomOmniManager
// https://sir.upc.edu/wikis/roblab/index.php/NewOmni/Cpp
// Topics and services
// Publishers

//  /poseMasterOutput
//   Current pose of the omni when the omni is unlocked and moving freely. When
//   locked it doesn't publish. External nodes must read from this topic. Type:
//   geometry_msgs::PoseStamped

//  /set_lock_pose
//   Position where to lock the Omni. This topic connects to the phantom_omni
//   node "set_lock_pose" topic. Type: geometry_msgs::Vector3

// Subscribers

//  /poseMasterInput
//   Current pose of the omni. This topic connects to the phantom_omni node
//   "pose" topic. Type: geometry_msgs::PoseStamped

//  /omni_locked
//   Current lock/unlock state of the Omni. This topic connects to the
//   phantom_omni node "lock" topic. Type: std_msgs::Bool

// Roslaunch example
// <node ns="omni" name="phantom_omni_manager" pkg="phantom_omni_manager"
// type="phantom_omni_manager">
//   <remap from="poseMasterInput" to="pose"/>
//   <remap from="poseMasterOutput" to="poseStampedMasterWorkspace"/>
//   <remap from="omni_locked" to="lock"/>
// </node>

// <node ns="omni" name="phantom_omni" pkg="phantom_omni" type="omni"/>