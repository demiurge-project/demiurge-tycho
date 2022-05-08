#include <ros/ros.h>
#include <std_msgs/Bool.h>

class CropperSupervisor
{
public:
	CropperSupervisor(ros::NodeHandle& nh, std::string ros_namespace)
		: alive(true)
	{
		std::string topic = ros_namespace + "/alive";
		sub_calibrator_ = nh.subscribe(topic, 10, &CropperSupervisor::supervisorCallback, this);
	}

	bool alive;

	void supervisorCallback(const std_msgs::BoolConstPtr& msg)
	{
		alive = msg->data;
	}

private:
	ros::Subscriber sub_calibrator_;
};

void supervisorCallback(const std_msgs::BoolConstPtr& msg) {}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle node_handle;

	CropperSupervisor supervisor_camera_0(node_handle, "/camera_0");
	CropperSupervisor supervisor_camera_1(node_handle, "/camera_1");
	CropperSupervisor supervisor_camera_2(node_handle, "/camera_2");

	ros::Rate rate(30);
	while (ros::ok() && (supervisor_camera_0.alive ||
	                     supervisor_camera_1.alive ||
	                     supervisor_camera_2.alive))
	{
		ros::spinOnce();
		rate.sleep();
	}

	ros::Duration(0.5).sleep();
	ROS_INFO("Cropper calibration done. Shutting down...");

	return 0;
}
