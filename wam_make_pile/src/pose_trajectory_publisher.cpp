/******************************************************************************
	           ROS 2 Cartesian Trajectory Generation Example
          Copyright (C) 2018, 2021 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/


//Inclui as bibliotecas do ROS 2, da biblioteca KDL (Kinematics and Dynamics Library) e das mensagens de pose do ROS 2
#include <rclcpp/rclcpp.hpp>

#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>
#include <tf2_kdl/tf2_kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

//Define o namespace KDL
using namespace KDL;

//Define a classe "PoseTrajectory" que herda da classe base "rclcpp::Node"
class PoseTrajectory: public rclcpp::Node
{
	public:
	PoseTrajectory(const char *name="trajectory_publisher");
	
	private:
	Trajectory_Composite trajectory_;
	double t0_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
	
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cokeSubscriber_;
	
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr jointPositionSub_;
	
	KDL::Frame goal_;
	
	KDL::Frame actual_;
	
	void cokeCB(const geometry_msgs::msg::PoseStamped::SharedPtr coke);
	
	void jointPositionCB(const geometry_msgs::msg::PoseStamped::SharedPtr jntPosition);
	
	void timerCB(void) const;
};

PoseTrajectory::PoseTrajectory(const char *name): Node(name)
{

	cokeSubscriber_=create_subscription<geometry_msgs::msg::PoseStamped>("coke_position",100,std::bind(&PoseTrajectory::cokeCB,this,std::placeholders::_1));
	
	posePublisher_=create_publisher<geometry_msgs::msg::PoseStamped>("pose",10);
	
	jointPositionSub_=create_subscription<geometry_msgs::msg::PoseStamped>("joint_position",100,std::bind(&PoseTrajectory::jointPositionCB,this,std::placeholders::_1));	
}


void PoseTrajectory::jointPositionCB(const geometry_msgs::msg::PoseStamped::SharedPtr jntPosition)
{
	actual_ = KDL::Frame(KDL::Rotation::Quaternion(jntPosition->pose.orientation.x, 
					     jntPosition->pose.orientation.y, 
					     jntPosition->pose.orientation.z, 
					     jntPosition->pose.orientation.w),
	KDL::Vector(jntPosition->pose.position.x,jntPosition->pose.position.y,jntPosition->pose.position.z));
}


void PoseTrajectory::cokeCB(const geometry_msgs::msg::PoseStamped::SharedPtr coke)
{
	goal_ = KDL::Frame(KDL::Rotation::Quaternion(coke->pose.orientation.x, 
					     coke->pose.orientation.y, 
					     coke->pose.orientation.z, 
					     coke->pose.orientation.w),
	KDL::Vector(coke->pose.position.x,coke->pose.position.y,coke->pose.position.z));
	
	
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"actual x " << actual_.p.x());
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"actual y " << actual_.p.y());
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"actual z " << actual_.p.z());
	
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"goal x " << goal_.p.x());
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"goal y " << goal_.p.y());
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"goal z " << goal_.p.z());
	
	
	//pegar a posição atual do robo com a KDL.
	
	try
	{
		auto path=new Path_Line(actual_, goal_, new RotationalInterpolation_SingleAxis(), 0.1);

		auto velocityProfile=new VelocityProfile_Trap(0.1,0.02);
		velocityProfile->SetProfile(0,path->PathLength());

		auto trajectorySegment=new Trajectory_Segment(path,velocityProfile);

		auto trajectoryStationary=new Trajectory_Stationary(1.0,goal_);
		

		trajectory_.Add(trajectorySegment);
		trajectory_.Add(trajectoryStationary);		
		
	}
	catch(Error &error)
	{
		RCLCPP_ERROR_STREAM(get_logger(),"Error: " << error.Description() << std::endl);
		RCLCPP_ERROR_STREAM(get_logger(),"Type: " << error.GetType() << std::endl);
	}
	
	t0_=now().seconds();
	using namespace std::chrono_literals;
	timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&PoseTrajectory::timerCB,this));
}

void PoseTrajectory::timerCB(void) const
{
	double t=fmin(now().seconds()-t0_,trajectory_.Duration());
	tf2::Stamped<KDL::Frame> pose(trajectory_.Pos(t),tf2::get_now(),"map");
	auto poseMsg=tf2::toMsg(pose);
	
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"x publicado: " << poseMsg.pose.position.x);
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"y publicado: " << poseMsg.pose.position.y);
	RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_trajectory_publisher"),"z publicado: " << poseMsg.pose.position.z);
	
	posePublisher_->publish(poseMsg);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PoseTrajectory>());
	rclcpp::shutdown();
	return 0;
}
