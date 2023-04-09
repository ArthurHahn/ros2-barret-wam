/******************************************************************************
	             ROS 2 pid_plus_gravity_controller Package
                        PID+Gravity Compensation Controller
          Copyright (C) 2017..2022 Walter Fetter Lages <w.fetter@ieee.org>

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

#include <sys/mman.h>

#include <rclcpp/logging.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <pid_plus_gravity_controller/pid_plus_gravity_controller.hpp>

namespace effort_controllers
{	
	PidPlusGravityController::PidPlusGravityController(void):
		q_(0),qr_(0),gravity_(0),referencePoint_(nullptr)
	{
	}

	controller_interface::CallbackReturn PidPlusGravityController::on_init(void)
	{
		try
		{
			auto_declare<std::vector<std::string>>("joints",jointNames_);

			auto_declare<std::string>("chain.root","origin_link");
			auto_declare<std::string>("chain.tip", "tool_link");

			auto_declare<double>("gravity.x",0.0);
			auto_declare<double>("gravity.y",0.0);
			auto_declare<double>("gravity.z",-9.8);

			auto_declare<int>("priority",sched_get_priority_max(SCHED_FIFO));
		}
		catch(const std::exception &e)
		{
			RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Exception thrown in on_init() with message: " << e.what());
			return controller_interface::CallbackReturn::ERROR;
		}

		rclcpp::QoS qos(rclcpp::KeepLast(1));
		qos.transient_local();
		auto robotDescriptionSubscriber=get_node()->create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&PidPlusGravityController::robotDescriptionCB,this,std::placeholders::_1));
		while(robotDescription_.empty())
		{
			RCLCPP_WARN_SKIPFIRST_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),1000,"Waiting for robot model on /robot_description.");
			rclcpp::spin_some(get_node()->get_node_base_interface());
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn PidPlusGravityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		jointNames_=get_node()->get_parameter("joints").as_string_array();
		if(jointNames_.empty())
		{
			RCLCPP_ERROR(get_node()->get_logger(),"'joints' parameter was empty,");
			return controller_interface::CallbackReturn::ERROR;
		}

		nJoints_=jointNames_.size();

		try
		{
			for(const auto &joint : jointNames_)
			{
				auto pid=std::make_shared<control_toolbox::PidROS>(get_node(),joint);
				pid->initPid();
				pid_.push_back(pid);
			}
		}
		catch(const std::exception &e)
		{
			RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Exception thrown while creating PidROS: " << e.what());
			return controller_interface::CallbackReturn::ERROR;
		}

		using std::placeholders::_1;
		sub_command_=get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("command",1,
		        std::bind(&PidPlusGravityController::commandCB,this,_1));

		while(robotDescription_.empty())
			RCLCPP_WARN_SKIPFIRST_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),1000,"Waiting for robot model on /robot_description.");

		try
		{
			KDL::Tree tree;
			if(!kdl_parser::treeFromString(robotDescription_,tree))
				throw std::runtime_error("Failed to construct KDL tree.");

			std::string chainRoot=get_node()->get_parameter("chain.root").as_string();
			if(chainRoot.empty())
				throw std::runtime_error("Could not find 'chain.root' parameter.");

			std::string chainTip=get_node()->get_parameter("chain.tip").as_string();
			if(chainTip.empty())
				throw std::runtime_error("Could not find 'chain,tip' parameter.");

			if(!tree.getChain(chainRoot,chainTip,chain_))
				throw std::runtime_error("Failed to get chain from KDL tree.");

			KDL::Vector g;
			g[0]=get_node()->get_parameter("gravity.x").get_value<double>();
			g[1]=get_node()->get_parameter("gravity.y").get_value<double>();
			g[2]=get_node()->get_parameter("gravity.z").get_value<double>();

			chainParam_=std::make_unique<KDL::ChainDynParam>(chain_,g);

			q_.resize(chain_.getNrOfJoints());
			qr_.resize(chain_.getNrOfJoints());
			gravity_.resize(chain_.getNrOfJoints());
		}
		catch(const std::exception &e)
		{
			RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Exception in on_configure(): " << e.what());
			return controller_interface::CallbackReturn::ERROR;
		}

		if(!get_node()->get_parameter("priority",priority_))
			RCLCPP_WARN(get_node()->get_logger(),"No 'priority' configured for controller. Using highest possible priority.");

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::InterfaceConfiguration PidPlusGravityController::command_interface_configuration(void) const
	{
		controller_interface::InterfaceConfiguration config;
		config.type=controller_interface::interface_configuration_type::INDIVIDUAL;

		for(const auto &joint : jointNames_)
			config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);

		return config;
	}

	controller_interface::InterfaceConfiguration PidPlusGravityController::state_interface_configuration(void) const
	{
		controller_interface::InterfaceConfiguration config;
		config.type=controller_interface::interface_configuration_type::INDIVIDUAL;

		for(const auto &joint : jointNames_)
			config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);

		return config;
	}

	controller_interface::CallbackReturn PidPlusGravityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		for(unsigned int i=0;i < nJoints_;i++)
			q_(i)=state_interfaces_[i].get_value();
                qr_=q_;

                struct sched_param param;
                param.sched_priority=priority_;
		if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
			RCLCPP_WARN(get_node()->get_logger(),"Failed to set real-time scheduler.");
		else if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
			RCLCPP_WARN(get_node()->get_logger(),"Failed to lock memory.");

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn PidPlusGravityController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
	{
		for(unsigned int i=0;i < nJoints_;i++)
			q_(i)=state_interfaces_[i].get_value();

		if(chainParam_->JntToGravity(q_,gravity_) < 0)
		        RCLCPP_ERROR(get_node()->get_logger(),"KDL dynamics solver failed.");

		auto time=get_node()->get_clock()->now();
		for(unsigned int i=0;i < nJoints_;i++)
		        command_interfaces_[i].set_value(gravity_(i));

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::return_type PidPlusGravityController::update(const rclcpp::Time &/*time*/,const rclcpp::Duration & period)
	{
		auto referencePoint=referencePoint_.readFromRT();
		if(referencePoint && *referencePoint)
			if((*referencePoint)->positions.size() == nJoints_)
				qr_.data=Eigen::VectorXd::Map((*referencePoint)->positions.data(),nJoints_);

		for(unsigned int i=0;i < nJoints_;i++)
			q_(i)=state_interfaces_[i].get_value();

		if(chainParam_->JntToGravity(q_,gravity_) < 0)
		        RCLCPP_ERROR(get_node()->get_logger(),"KDL dynamics solver failed.");

		for(unsigned int i=0;i < nJoints_;i++)
		        command_interfaces_[i].set_value(gravity_(i)+pid_[i]->computeCommand(qr_(i)-q_(i),period));

		return controller_interface::return_type::OK;
	}

	void PidPlusGravityController::commandCB(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr referencePoint)
	{
		referencePoint_.writeFromNonRT(referencePoint);
	}

	void PidPlusGravityController::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
	{
		robotDescription_=robotDescription->data;
	}
}

PLUGINLIB_EXPORT_CLASS(effort_controllers::PidPlusGravityController,controller_interface::ControllerInterface)
