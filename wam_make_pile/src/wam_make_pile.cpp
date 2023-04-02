/*
  Q4d_teleop_rviz: A ROS 2 node to teloperate the Quanser 2DSFJE robot.
  
  Copyright (c) 2018, 2019, 2021 Walter Fetter Lages <w.fetter@ieee.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    You can also obtain a copy of the GNU General Public License
    at <http://www.gnu.org/licenses>.

*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <thread>
#include <chrono>

#define sqr(x) ((x)*(x))

class Q4dTeleop: public rclcpp::Node
{
	public:
	Q4dTeleop(void);
	~Q4dTeleop(void);
	void publish(void);
			
	private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clSub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jsPub_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hSub_; 
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grpSub_; 
	
	KDL::Frame goal_;
	
	double row;
	double pitch;
	double yaw;
	double h_;
	double h = 0;
	double c_;
	double c = 0;

	sensor_msgs::msg::JointState joint_state;

	std::string robotDescription_;
	KDL::Chain chain_;
	KDL::ChainIkSolverPos_LMA *ikSolverPos_;
	KDL::JntArray q_;
		
	void clickCB(const geometry_msgs::msg::PoseStamped::SharedPtr click);
	void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
	void hCB(const std_msgs::msg::Float64::SharedPtr msg);
	void cCB(const std_msgs::msg::Float64::SharedPtr msg);
};

Q4dTeleop::Q4dTeleop(void): Node("Q4d_teleop_rviz"), q_(2)
{
	clSub_=create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",100,std::bind(&Q4dTeleop::clickCB,this,std::placeholders::_1));
	jsPub_=create_publisher<sensor_msgs::msg::JointState>("joint_states",100); 
	hSub_=create_subscription<std_msgs::msg::Float64>("height",100,std::bind(&Q4dTeleop::hCB,this,std::placeholders::_1));
	grpSub_=create_subscription<std_msgs::msg::Float64>("gripper",100,std::bind(&Q4dTeleop::cCB,this,std::placeholders::_1));  
	
	joint_state.name.resize(5);
	
	joint_state.position.resize(5);
	
	joint_state.name[0] = "shoulder_active_joint";
	joint_state.name[1] = "elbow_active_joint";
	joint_state.name[2] = "prismatic_joint";
	joint_state.name[3] = "revolute_joint";
	joint_state.name[4] = "claw_joint"; 	
	
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();
	auto robotDescriptionSubscriber_=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&Q4dTeleop::robotDescriptionCB,this,std::placeholders::_1));
	while(robotDescription_.empty())
	{
                RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
                rclcpp::spin_some(get_node_base_interface());
	}

	KDL::Tree tree;
	if (!kdl_parser::treeFromString(robotDescription_,tree))
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
		
	if (!tree.getChain("origin_link","revolute_link",chain_))
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");

	q_.resize(chain_.getNrOfJoints());

	//goal_.Identity();
	
	Eigen::Matrix<double,6,1> L;
	L << 1.0 , 1.0 , 1.0, 0.01, 0.01, 0.01;

	// A copy of chain_ is not created inside!
	ikSolverPos_=new KDL::ChainIkSolverPos_LMA(chain_,L);
	ikSolverPos_->display_information=false;
}

Q4dTeleop::~Q4dTeleop(void)
{
	delete ikSolverPos_;
}

void Q4dTeleop::clickCB(const geometry_msgs::msg::PoseStamped::SharedPtr click)
{
	goal_ = KDL::Frame(KDL::Rotation::Quaternion(click->pose.orientation.x, 
                                                 click->pose.orientation.y, 
                                                 click->pose.orientation.z, 
                                                 click->pose.orientation.w),
                       	  KDL::Vector(click->pose.position.x,click->pose.position.y,click->pose.position.z));

    goal_.M.GetRPY(row,pitch,yaw);
	//goal_.p.x(click->point.x);
	//goal_.p.y(click->point.y);
	//goal_.p.z(0.1477);
	//goal_.M.RotZ(atan2(click->point.y,click->point.x));
	
	publish();
}

void Q4dTeleop::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
	robotDescription_=robotDescription->data;
}

void Q4dTeleop::hCB(const std_msgs::msg::Float64::SharedPtr msg){ //interpolação altura
	
	h_ = msg->data;
	double h_delta;
	h_delta = (h_- h)/100;
	
	for(int j = 0; j<=100; j++){

		joint_state.header.stamp = now();
		joint_state.position[2] = h + h_delta*j;
		jsPub_->publish(joint_state);
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

	}

	h = h_;
	
}

void Q4dTeleop::cCB(const std_msgs::msg::Float64::SharedPtr msg){ //interpolação garra

	c_ = msg->data;
	double c_delta;
	c_delta = (c_- c)/100;
	
	for(int j = 0; j<=100; j++){

		joint_state.header.stamp = now();
		joint_state.position[4] = c + c_delta*j;
		jsPub_->publish(joint_state);
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

	}

	c = c_;

}

//#define KDL_IK
void Q4dTeleop::publish(void)
{
//#ifdef KDL_IK
	KDL::JntArray q_o=q_;
	int error=0;
	if((error=ikSolverPos_->CartToJnt(q_,goal_,q_o)) < 0)
		RCLCPP_ERROR_STREAM(get_logger(),"Failed to compute invere kinematics: (" << error << ") "
			<< ikSolverPos_->strError(error));
	// q_=q_out;
	
	double g_yaw = yaw - q_o(0) - q_o(1);
	double qzero = (q_o(0) - q_(0))/100;
	double qum = (q_o(1) - q_(1))/100;
	double yaw_diff = (g_yaw - yaw)/100;

	for(int j = 0; j<=100; j++){

		joint_state.header.stamp = now();
		joint_state.position[0] = q_(0) + qzero * j;
		joint_state.position[1] = q_(1) + qum * j;
		joint_state.position[3] = yaw + yaw_diff * j;
		
		jsPub_->publish(joint_state);
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

	}
 
	q_=q_o;
	
//#else
	// Algebric inverse kinematics
//	const double l1=0.343;
//	const double l2=0.267;
//	double c1=(sqr(goal_.p.x())+sqr(goal_.p.y())-sqr(l1)-sqr(l2))/2/l1/l2;
//	if(c1 >= 0.0 && c1 <= 1.0)
//	{
//		double s1a=sqrt(1-sqr(c1));
//		double s1b=-s1a;
//		double q1a=atan2(s1a,c1);
//		double q1b=atan2(s1b,c1);
      
//		double q0a=atan2(goal_.p.y(),goal_.p.x())-atan2(l2*s1a,l1+l2*c1);
//		double q0b=atan2(goal_.p.y(),goal_.p.x())-atan2(l2*s1b,l1+l2*c1);
      
//		if((fabs(q_(0)-q0a) < fabs(q_(0)-q0b)) || (cos(q0a) > 0.0))
//		{
//			q_(0)=q0a;
//			q_(1)=q1a;
//		}
//		else
//		{
//			q_(0)=q0b;
//			q_(1)=q1b;
//		}
//	}
//#endif

	//std_msgs::msg::Float64 shoulderCmd;
	//std_msgs::msg::Float64 elbowCmd;
	
	//shoulderCmd.data=q_(0);
	//elbowCmd.data=q_(1);
	
	//shoulderCmdPub_->publish(shoulderCmd);
	//elbowCmdPub_->publish(elbowCmd);
}


int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Q4dTeleop>());
	rclcpp::shutdown();
	return 0;
}
