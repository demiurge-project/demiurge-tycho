#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <cmath>

class TransformerCalibrator
{
public:
	TransformerCalibrator()
		: done(false)
	{
		sub_tracker_ = node_handle_.subscribe("tracker/positions_stamped", 10, 
		                                      &TransformerCalibrator::calibratorCallback, this);
		pub_alive_ = node_handle_.advertise<std_msgs::Bool>("alive", 10);

		// Open YAML file to store the calibration parameters
		std::string path = ros::package::getPath("transformer") + "/config";
		std::string ros_namespace = ros::this_node::getNamespace();
		std::string filename = path + ros_namespace + "_transformer.yaml";
		yaml_.open(filename);
		if (yaml_.is_open())
		{
			ROS_INFO("Writing configuration to %s", filename.c_str());
		}
		else
		{
			ROS_ERROR("Could not open %s", filename.c_str());
		}
	}

	~TransformerCalibrator()
	{
		yaml_.close();
	}

	struct point
	{
		double x;
		double y;
	};

	bool done;

	// calibration_tags holds the IDs of the 9 tags used for calibration.
	// The first tag is the one in the centre and the rest are given anti-clockwise,
	// starting from 0 degrees with respect to the x-axis of the arena.
	// Currently hardcoded and initialised in the constructor
	const std::vector<int> calibration_tags = {38, 17, 37, 23, 31, 40, 22, 10, 45};

	// grid_dim is the radius of the calibration board
	const double grid_dim = 0.1185;

	void calibratorCallback(const std_msgs::UInt32MultiArrayConstPtr& msg)
	{
		// Process incoming message
		int size = msg->data.size();

		if (size == 83) // 2 for timestamp and 9 * 9 = 81 for the 9 entires of the 9 tags (ID and x-y coordinates of each corner)
		{
			std::map<int, struct point> tags; // holds the ID and position of each tag
			for (int i = 2; i < size; i += 9)
			{
				// Calculate position of the centre of each tag
				struct point tag_centre = {0.0};
				for (int j = 0; j < 4; ++j)
				{
					// Transform left-handed (camera) frame to right-handed (arena) frame
					tag_centre.x -= static_cast<double>(msg->data.at(2*j + 1 + i)) / 4.0;
					tag_centre.y += static_cast<double>(msg->data.at(2*j + 2 + i)) / 4.0;
				}

				// Add tag to map
				tags[msg->data.at(i)] = tag_centre;
			}

			// Calculate angular offset and scale factor
			double angular_offset = 0.0;
			double scale_factor = 0.0;
			for (int i = 0; i < 4; ++i)
			{
				double x1 = tags[calibration_tags.at(i+5)].x;
				double y1 = tags[calibration_tags.at(i+5)].y;
				double x2 = tags[calibration_tags.at(i+1)].x;
				double y2 = tags[calibration_tags.at(i+1)].y;

				// Angular offset
				double orientation = std::atan2(y2 - y1, x2 - x1);
				angular_offset += (orientation - i * M_PI_4) / 4.0;

				// Scale factor
				double size = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
				scale_factor += size / 4.0;
			}

			scale_factor = grid_dim * 2.0 / scale_factor;

			// Write YAML file
			yaml_ << "x_offset : " << tags[calibration_tags.at(0)].x << "\n";
			yaml_ << "y_offset : " << tags[calibration_tags.at(0)].y << "\n";
			yaml_ << "angular_offset : " << angular_offset << "\n";
			yaml_ << "scale_factor : " << scale_factor << "\n";

			done = true;
		}

		// Let the supervisor node if calibration is still ongoing
		std_msgs::Bool alive_msg;
		alive_msg.data = !done;
		pub_alive_.publish(alive_msg);
	}

private:
	ros::NodeHandle node_handle_;
	ros::Subscriber sub_tracker_;
	ros::Publisher pub_alive_;
	std::ofstream yaml_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrator");
	TransformerCalibrator calibrator_object;

	ros::Rate rate(30);
	while (ros::ok() && !calibrator_object.done)
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
