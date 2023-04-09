/******************************************************************************
	               ROS 2 PoseStamped to tf Converter
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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class PoseStampedToTf: public rclcpp::Node
{
	public:
	PoseStampedToTf(const char *node_name,const char *child_frame_id);

	private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSubscriber_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPublisher_;
	std::string child_frame_id_;
	void poseStampedCB(const geometry_msgs::msg::PoseStamped::SharedPtr poseMsg) const;
};

PoseStampedToTf::PoseStampedToTf(const char *node_name,const char *child_frame_id): Node(node_name)
{
	child_frame_id_=child_frame_id;
	poseSubscriber_=create_subscription<geometry_msgs::msg::PoseStamped>("pose",10,std::bind(&PoseStampedToTf::poseStampedCB,this,std::placeholders::_1));
	tfPublisher_=create_publisher<tf2_msgs::msg::TFMessage>("/tf",10);
}

void PoseStampedToTf::poseStampedCB(const geometry_msgs::msg::PoseStamped::SharedPtr poseMsg) const
{
	tf2_msgs::msg::TFMessage tfMsg;

	tfMsg.transforms.resize(1);
	tfMsg.transforms[0].header.stamp=poseMsg->header.stamp;
	tfMsg.transforms[0].header.frame_id=poseMsg->header.frame_id;
	tfMsg.transforms[0].child_frame_id=child_frame_id_;
	tfMsg.transforms[0].transform.translation.x=poseMsg->pose.position.x;
	tfMsg.transforms[0].transform.translation.y=poseMsg->pose.position.y;
	tfMsg.transforms[0].transform.translation.z=poseMsg->pose.position.z;
	tfMsg.transforms[0].transform.rotation=poseMsg->pose.orientation;
	
        tfPublisher_->publish(tfMsg);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	
	if(argc < 2)
	{
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("pose_stamped2tf"),"pose_stamped2tf: No child frame id.\n");
		return -1;
	}
	
	rclcpp::spin(std::make_shared<PoseStampedToTf>("pose_stamped2tf",argv[1]));
	
	return 0;
}
