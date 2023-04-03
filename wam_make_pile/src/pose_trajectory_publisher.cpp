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

#include <rclcpp/rclcpp.hpp>

#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>
#include <tf2_kdl/tf2_kdl.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace KDL;

class PoseTrajectory: public rclcpp::Node
{
	public:
	PoseTrajectory(const char *name="trajectory_publisher");
	
	private:
	Trajectory_Composite trajectory_;
	double t0_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
	
	void timerCB(void) const;
};

PoseTrajectory::PoseTrajectory(const char *name): Node(name)
{
	try
	{
		auto path=new Path_RoundedComposite(0.02,0.002,new RotationalInterpolation_SingleAxis());
		path->Add(Frame(Rotation::RPY(0,0,0),Vector(0.61,0,0.1477)));
		path->Add(Frame(Rotation::Quaternion(0,0,-0.375,0.926),Vector(0.437,-0.424,0.1477)));
		path->Add(Frame(Rotation::RotZ(-M_PI_2),Vector(0.238,-0.505,0.1477)));
		path->Add(Frame(Rotation::Quaternion(0,0,0.375,0.926),Vector(0.437,0.424,0.1477)));
		path->Add(Frame(Rotation::RotZ(M_PI_2*0),Vector(0.238,0.505,0.1477)));
		path->Add(Frame(Rotation::RPY(0,0,0),Vector(0.61,0,0.1477)));
		path->Finish();

		auto velocityProfile=new VelocityProfile_Trap(0.1,0.02);
		velocityProfile->SetProfile(0,path->PathLength());

		auto trajectorySegment=new Trajectory_Segment(path,velocityProfile);

		auto trajectoryStationary=new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0),Vector(0.61,0,0.1477)));

		trajectory_.Add(trajectorySegment);
		trajectory_.Add(trajectoryStationary);
	}
	catch(Error &error)
	{
		RCLCPP_ERROR_STREAM(get_logger(),"Error: " << error.Description() << std::endl);
		RCLCPP_ERROR_STREAM(get_logger(),"Type: " << error.GetType() << std::endl);
	}

	posePublisher_=create_publisher<geometry_msgs::msg::PoseStamped>("pose",10);

	t0_=now().seconds();
	using namespace std::chrono_literals;
	timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&PoseTrajectory::timerCB,this));
}

void PoseTrajectory::timerCB(void) const
{
	double t=fmin(now().seconds()-t0_,trajectory_.Duration());
	tf2::Stamped<KDL::Frame> pose(trajectory_.Pos(t),tf2::get_now(),"map");
	auto poseMsg=tf2::toMsg(pose);
	posePublisher_->publish(poseMsg);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PoseTrajectory>());
	rclcpp::shutdown();
	return 0;
}
