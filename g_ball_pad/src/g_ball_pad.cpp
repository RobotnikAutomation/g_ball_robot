/*
 * guardian_pad
 * Copyright (c) 2011, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation SLL
 * \brief Allows to use a pad with a LWA4P, sending messages to a joint_trajectory_controller interface
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/ptz.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <guardian_pad/enable_disable.h>

#define DEFAULT_NUM_OF_BUTTONS		20
#define DEFAULT_AXIS_LINEAR			1
#define DEFAULT_AXIS_ANGULAR		2	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		2.0

#define DEFAULT_MAX_LINEAR_SPEED	2.0		// m/s
#define DEFAULT_MAX_ANGULAR_SPEED	40.0	// rads /s

#define JOINT_MIN_ANGLE             -2.9
#define JOINT_MAX_ANGLE             2.9


class GuardianPad
{
        public:
	GuardianPad();
	//! Updates diagnostics
	void Update();

	private:
	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
	bool EnableDisable(guardian_pad::enable_disable::Request &req, guardian_pad::enable_disable::Response &res );
	void publishJointTrajectory(std::vector<double> joint_positions, double time_from_start);
	
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	
	ros::Publisher joint_trajectory_pub_;
	//! It will be suscribed to the joystick
	ros::Subscriber pad_sub_;

    ros::Subscriber joint_states_sub_;

	double current_vel;
	//! Number of the DEADMAN button
	int dead_man_button_;
	int dead_man_button_lwa4p_;
	//! Number of the button for increase or decrease the selected joint
	int next_joint_button_; 
    int previous_joint_button_;
	int button_output_1_, button_output_2_;
	int output_1_, output_2_;
	bool bOutput1, bOutput2;
	//! buttons to the pan-tilt-zoom camera
	int ptz_tilt_up_, ptz_tilt_down_, ptz_pan_right_, ptz_pan_left_;
	int ptz_zoom_wide_, ptz_zoom_tele_;	
	//! Service to modify the digital outputs
	ros::ServiceClient set_digital_outputs_client_;  
	//! Enables/disables the pad
	ros::ServiceServer enable_disable_srv_;
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; // 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; // 	
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;
	//! Pan & tilt increment (degrees)
	int pan_increment_, tilt_increment_;
	//! Zoom increment (steps)
	int zoom_increment_;
    
    std::vector<std::string> joint_names_;
    
    int selected_joint_;
    sensor_msgs::JointState joint_states_msg_;
    
    struct joint_limit
    {
		double min;
		double max;
	};
	
	std::vector<joint_limit> joint_limits_;
};


GuardianPad::GuardianPad():
  linear_(1),
  angular_(2)
{

	pnh_ = ros::NodeHandle("~");
	current_vel = 0.1;

	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

	// MOTION CONF
	nh_.param("axis_linear", linear_, DEFAULT_AXIS_LINEAR);
	nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
	nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
	nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
	nh_.param("button_dead_man_lwa4p", dead_man_button_lwa4p_, 9);
	nh_.param("button_next_joint", next_joint_button_, 4);
	nh_.param("button_previous_joint", previous_joint_button_, 5);
	

	for(int i = 0; i < DEFAULT_NUM_OF_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}


    // Initialize joint_names_
    joint_names_.push_back(std::string("arm_1_joint"));
    joint_names_.push_back(std::string("arm_2_joint"));
    joint_names_.push_back(std::string("arm_3_joint"));
    joint_names_.push_back(std::string("arm_4_joint"));
    joint_names_.push_back(std::string("arm_5_joint"));
    joint_names_.push_back(std::string("arm_6_joint"));
    joint_names_.push_back(std::string("pg70_finger_left_joint"));
    
    // Initialize joint_limits_
    for(int i=0; i<joint_names_.size(); i++)
    {
		joint_limit limit;
		if(joint_names_[i]!=std::string("pg70_finger_left_joint"))
		{
			limit.min = JOINT_MIN_ANGLE;
			limit.max = JOINT_MAX_ANGLE;
		}
		else // Custom limit for PG70 joint
		{
			limit.min = 0.0045;
			limit.max = 0.0385;
		}
		joint_limits_.push_back(limit);
	}
    
    selected_joint_ = 0;
	ROS_INFO("GBallPad::padCallback: selected joint %s", joint_names_[selected_joint_].c_str());



 	// Listen through the node handle sensor_msgs::Joy messages from joystick (these are the orders that we will send to guardian_controller/command)
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &GuardianPad::padCallback, this);
	joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &GuardianPad::jointStatesCallback, this);
	
	joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);

	// Advertises new service to enable/disable the pad
	enable_disable_srv_ = pnh_.advertiseService("enable_disable",  &GuardianPad::EnableDisable, this);
	
	// Diagnostics
	updater_pad.setHardwareID("None");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));


	bEnable = true;	// Communication flag enabled by default
	
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void GuardianPad::Update(){
	updater_pad.update();
}

/*
 *	\brief Enables/Disables the pad
 *
 */
bool GuardianPad::EnableDisable(guardian_pad::enable_disable::Request &req, guardian_pad::enable_disable::Response &res )
{
	bEnable = req.value;

	ROS_INFO("GuardianPad::EnablaDisable: Setting to %d", req.value);
	res.ret = true;
	return true;
}
	  
/*
 *	\brief Method call when receiving a message from the topic /joy
 *
 */
void GuardianPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std::vector<double> joint_positions; // positions ordered as joint_names_ vector
	
	// Check availavility of joint_states msg
	//ROS_INFO("now: %d, joint_states_: %d", ros::Time::now().sec, joint_states_msg_.header.stamp.sec);
	if( (ros::Time::now() - joint_states_msg_.header.stamp) > ros::Duration(1) )
	{
		ROS_ERROR_THROTTLE(1, "GBallPad::padCallback: joint_states msg is too old");
		return;
	}
	
	// Check if we have a JointState msg with all the defined joints
	for(int i=0; i<joint_names_.size(); i++)
	{
		bool joint_found = false;
		for(int j=0; j<joint_states_msg_.name.size(); j++)
		{
			if(joint_states_msg_.name[j] == joint_names_[i])
			{
				joint_found = true;
				joint_positions.push_back(joint_states_msg_.position[j]);
				break;
			}
		}
		if(!joint_found)
		{
			ROS_ERROR_THROTTLE(1, "GBallPad::padCallback: one or more joints can't be found in joint_states topic");
			return;
		}
	}
	//ROS_INFO("joint_positions size %d", (int)joint_positions.size());

	
	if(joy->buttons[dead_man_button_lwa4p_] == 1 && joy->buttons[dead_man_button_] != 1)
	{
		//ROS_INFO("Dead man button LWA4P pressed");
		
		// Next joint button
		if(joy->buttons[next_joint_button_] == 1 && !bRegisteredButtonEvent[next_joint_button_])
		{
			bRegisteredButtonEvent[next_joint_button_] = true;
			selected_joint_++;
			if(selected_joint_ == joint_names_.size())
				selected_joint_ = 0;
			ROS_INFO("GBallPad::padCallback: selected joint %s", joint_names_[selected_joint_].c_str());
		}
		else if( joy->buttons[next_joint_button_] != 1)
			bRegisteredButtonEvent[next_joint_button_] = false;
		
		// Previous joint button
		if(joy->buttons[previous_joint_button_] == 1 && !bRegisteredButtonEvent[previous_joint_button_])
		{
			bRegisteredButtonEvent[previous_joint_button_] = true;
			selected_joint_--;
			if(selected_joint_ < 0)
				selected_joint_ = joint_names_.size()-1;
			ROS_INFO("GBallPad::padCallback: selected joint %s", joint_names_[selected_joint_].c_str());
		}
		else if(joy->buttons[previous_joint_button_] != 1)
			bRegisteredButtonEvent[previous_joint_button_] = false;
			
			
		std::vector<double> trajectory;
		float desired_speed = l_scale_*joy->axes[linear_];
		if (desired_speed == 0.0)
		{
			// Stop movement
			publishJointTrajectory(joint_positions, 0.5); // Send current position
			return;	
		}
			
		
		double distance;
		double desired_position;
		if(desired_speed > 0)
			desired_position = joint_limits_[selected_joint_].max;
		else if(desired_speed < 0)
			desired_position = joint_limits_[selected_joint_].min;
		distance = desired_position - joint_positions[selected_joint_];
		
		double time_from_start;
		if(desired_speed != 0.0)
			time_from_start = distance / desired_speed;
		else
			time_from_start = 0.0;
		
		
		//ROS_INFO("Desired position: %f, current position: %f, distance: %f", desired_position, joint_positions[selected_joint_], distance);
		//ROS_INFO("Desired_speed: %f, time_from_start: %f", desired_speed, time_from_start);
		
		// Publish trajectory
		trajectory = joint_positions;
		trajectory[selected_joint_] = desired_position;
		publishJointTrajectory(trajectory, time_from_start);
	}
	else
	{
		// Stop movement
		publishJointTrajectory(joint_positions, 0.5); // Send current position
	}

}

void GuardianPad::publishJointTrajectory(std::vector<double> joint_positions, double time_from_start)
{
	if(!bEnable)
		return;

	trajectory_msgs::JointTrajectory joint_trajectory;
	joint_trajectory.joint_names = joint_names_;
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions = joint_positions;
	for (int i=0; i<joint_names_.size(); i++)
	{
		point.velocities.push_back(0.0);
		point.accelerations.push_back(0.0);
		point.effort.push_back(0.0);
	}
	
	if(time_from_start < 0.0)
		time_from_start = 0.0;
	point.time_from_start = ros::Duration(time_from_start);
	joint_trajectory.points.push_back(point);
	
	// Publish joint trajectory
	joint_trajectory_pub_.publish(joint_trajectory);
	
	return;
}

void GuardianPad::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_states_msg_ = *msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "g_ball_pad");
	GuardianPad guardian_pad;

	ros::Rate r(50.0);

	while( ros::ok() ){

		// UPDATING DIAGNOSTICS
		guardian_pad.Update();
		ros::spinOnce();
		r.sleep();
	}


}

