/*
  pose_stamped2joint: A ROS 2 node to map Cartesian pose to joint space
  
  Copyright (c) 2018, 2021 Walter Fetter Lages <w.fetter@ieee.org>

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
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>

class Pose2Joint: public rclcpp::Node
{
	public:
	Pose2Joint(const std::string &name,const std::string &root,const std::string &tip,const Eigen::Matrix<double,6,1> &L);
			
	private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr jointTrajPointPub_;

	std::string robotDescription_;
	KDL::Chain chain_;
	std::unique_ptr<KDL::ChainIkSolverPos_LMA> ikSolverPos_;
	KDL::JntArray q_;
        builtin_interfaces::msg::Time::UniquePtr t0_;
		
	void poseCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
	void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
};

Pose2Joint::Pose2Joint(const std::string &name,const std::string &root,const std::string &tip,const Eigen::Matrix<double,6,1> &L): Node(name), q_(0)
{
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        auto robotDescriptionSubscriber_=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&Pose2Joint::robotDescriptionCB,this,std::placeholders::_1));
        while(robotDescription_.empty())
        {
                RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
                rclcpp::spin_some(get_node_base_interface());
        }

	KDL::Tree tree;
	if(!kdl_parser::treeFromString(robotDescription_,tree)) RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
	if(!tree.getChain(root,tip,chain_)) RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");
	ikSolverPos_=std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_,L);

	q_.resize(chain_.getNrOfJoints());
        
	jointTrajPointPub_=create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("joint_trajectory_point",10);
	poseSub_=create_subscription<geometry_msgs::msg::PoseStamped>("/pose",10,std::bind(&Pose2Joint::poseCB,this,std::placeholders::_1));
}

void Pose2Joint::poseCB(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
{
	tf2::Stamped<KDL::Frame> goalFrame;
	tf2::fromMsg(*poseStamped,goalFrame);

	int error=ikSolverPos_->CartToJnt(q_,goalFrame,q_);
	if(error != 0) RCLCPP_ERROR_STREAM(get_logger(),"Failed to compute invere kinematics: (" << error << ") "
	        << ikSolverPos_->strError(error));

	trajectory_msgs::msg::JointTrajectoryPoint jointTrajPoint;
	jointTrajPoint.positions.resize(q_.rows());
	Eigen::VectorXd::Map(&jointTrajPoint.positions[0],jointTrajPoint.positions.size())=q_.data;

	if(!t0_) t0_=std::make_unique<builtin_interfaces::msg::Time>(poseStamped->header.stamp);
	if(poseStamped->header.stamp.nanosec >= t0_->nanosec)
	{
	        jointTrajPoint.time_from_start.nanosec=poseStamped->header.stamp.nanosec-t0_->nanosec;
	        jointTrajPoint.time_from_start.sec=poseStamped->header.stamp.sec-t0_->sec;
        }
        else
        {
	        jointTrajPoint.time_from_start.nanosec=1000000000+poseStamped->header.stamp.nanosec-t0_->nanosec;
	        jointTrajPoint.time_from_start.sec=poseStamped->header.stamp.sec-t0_->sec-1;
        }

	jointTrajPointPub_->publish(jointTrajPoint);
}

void Pose2Joint::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
	robotDescription_=robotDescription->data;
}

int main(int argc,char* argv[])
{
        rclcpp::init(argc,argv);
        if(argc < 3)
        {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_stamped2joint"),"Please, provide a chain root and a chain tip");
                return -1;
        }
        
        Eigen::Matrix<double,6,1> L;
        L << 1.0 , 1.0 , 1.0, 0.01, 0.01, 0.01;
        for(int i=0;i < argc-3 && i < L.size();i++) L(i)=atof(argv[i+3]);
        
        rclcpp::spin(std::make_shared<Pose2Joint>("pose_stamped2joint",argv[1],argv[2],L));
        
        return 0;
}
