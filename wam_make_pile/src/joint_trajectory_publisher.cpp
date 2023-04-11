/******************************************************************************
	           ROS 2 Joint Trajectory Generation Example
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

#include <rclcpp/rclcpp.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace KDL;

class JointTrajectory: public rclcpp::Node
{
	public:
	JointTrajectory(std::vector<double> &p0,std::vector<double> &pf,double tf,const char *name="trajectory_publisher");
	
	private:
	std::vector<VelocityProfile_Spline> velocityProfile_;
	double t0_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr jointPublisher_;
	
	void timerCB(void) const;
};

JointTrajectory::JointTrajectory(std::vector<double> &p0,std::vector<double> &pf,double tf,const char *name): Node(name)
{
	velocityProfile_.resize(min(p0.size(),pf.size()));
	for(unsigned int i=0;i < velocityProfile_.size();i++)
		velocityProfile_[i].SetProfileDuration(p0[i],0,0,pf[i],0,0,tf);

	jointPublisher_=create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("joint_trajectory_point",10);

	t0_=now().seconds();
	using namespace std::chrono_literals;
	timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&JointTrajectory::timerCB,this));
}

void JointTrajectory::timerCB(void) const
{
	double t=fmin(now().seconds()-t0_,velocityProfile_[0].Duration());
	
	trajectory_msgs::msg::JointTrajectoryPoint jointMsg;
	for(auto const &velocityProfile : velocityProfile_)
	{
		jointMsg.positions.push_back(velocityProfile.Pos(t));
		jointMsg.velocities.push_back(velocityProfile.Vel(t));
		jointMsg.accelerations.push_back(velocityProfile.Acc(t));
	}

	double sec;
	jointMsg.time_from_start.nanosec=modf(t,&sec)*1e9;
	jointMsg.time_from_start.sec=sec;
	
	if(t>=trajectory_.Duration())
	{
		jointPublisher_->publish(poseMsg);
	}
}	

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	
	std::vector<double> p0 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> pf {0.0, -2.0, 0.0, 3.1, 0.0, 0.0, 0.0};
	rclcpp::spin(std::make_shared<JointTrajectory>(p0,pf,7.0));
	
	rclcpp::shutdown();
	return 0;
}
