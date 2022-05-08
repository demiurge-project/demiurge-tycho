/* https://stackoverflow.com/questions/42190687/opencv-how-can-i-select-a-region-of-image-irregularly-with-mouse-event-c-c */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

class CropperCalibrator
{
public:
	CropperCalibrator()
		: image_transport_(node_handle_), cropped(false), n_points(0), ratio_(0.8)
	{
		image_sub_ = image_transport_.subscribe("camera/image_rect_color", 1,
		                                        &CropperCalibrator::imageCallback, this);
		pub_alive_ = node_handle_.advertise<std_msgs::Bool>("alive", 10);
		
		// Open YAML file to store the calibration parameters
		std::string path = ros::package::getPath("tycho_cropper") + "/config";
		std::string ros_namespace = ros::this_node::getNamespace();
		std::string filename = path + ros_namespace + "_cropper.yaml";
		yaml_.open(filename, std::fstream::out | std::fstream::trunc);
		if (yaml_.is_open())
		{
			ROS_INFO("Writing configuration to %s", filename.c_str());
		}
		else
		{
			ROS_ERROR("Could not open %s", filename.c_str());
		}

		// Get resizing ratio parameter
		node_handle_.getParam("calibrator/ratio", ratio_);
		if (ratio_ < 0.0)
		{
			ROS_WARN("Resizing ratio must be positive. Using absolute value");
			ratio_ = -ratio_;
		}
	}

	~CropperCalibrator()
	{
		yaml_.close();
	}

	bool cropped;
	unsigned int n_points;

	static void mouseCallback(int event, int x, int y, int flags, void* userdata)
	{
		// Should perhaps check if userdata is not nullptr
		CropperCalibrator* calibrator = reinterpret_cast<CropperCalibrator*>(userdata);
		calibrator->mouseCallback(event, x, y); // Calls class callback function, which is not static
	}

	void mouseCallback(int event, int x, int y)
	{
		if(event == cv::EVENT_RBUTTONDOWN)
		{
			std::cout << "Right mouse button clicked at (" << x << ", " << y << ")" << std::endl;
			if (vertices_.size() < 2)
			{
				std::cout << "You need a minimum of three points!" << std::endl;
				return;
			}

			// Close polygon
			cv::line(img_resize_, vertices_[vertices_.size()-1], vertices_[0], cv::Scalar(0, 0, 0));

			cropped = true;

			return;
		}

		if (event == cv::EVENT_LBUTTONDOWN)
		{
			std::cout << "Left mouse button clicked at (" << x << ", " << y << ")" << std::endl;
			if (vertices_.size() == 0)
			{
				// First click - just draw point
				img_resize_.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 0, 0);
			}
			else
			{
				// Second, or later click, draw line to previous vertex
				cv::line(img_resize_, cv::Point(x, y), vertices_[vertices_.size()-1], cv::Scalar(0, 0, 0));
			}

			vertices_.push_back(cv::Point(x, y));

			// Recover pixels in original image
			int x_img = x / ratio_;
			int y_img = y / ratio_;

			// Append to YAML file
			yaml_ << "point_" << n_points << " : [" << x_img << ", " << y_img << "]\n";
			n_points++;

			return;
		}
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		// Store image for use in cropping algorithm
		img_ = cv_ptr->image;
		cv::resize(img_, img_resize_, cv::Size(), ratio_, ratio_, cv::INTER_AREA);

		// If polygon has not been delimited start cropping process
		if (!cropped)
		{
			// Create a window
			cv::namedWindow("ImageDisplay", cv::WINDOW_AUTOSIZE);

			// Register a mouse callback
			cv::setMouseCallback("ImageDisplay", mouseCallback, this);

			// Cropped is false until a closed polygon has been defined
			while (!cropped)
			{
				cv::imshow("ImageDisplay", img_resize_);
				cv::waitKey(50);
			}

			cv::destroyWindow("ImageDisplay");
		}

		// Let the supervisor node if calibration is still ongoing
		std_msgs::Bool alive_msg;
		alive_msg.data = !cropped;
		pub_alive_.publish(alive_msg);
	}

private:
	ros::NodeHandle node_handle_;
	image_transport::ImageTransport image_transport_;
	image_transport::Subscriber image_sub_;
	ros::Publisher pub_alive_;
	std::ofstream yaml_;
	cv::Mat img_, img_resize_;
	std::vector<cv::Point> vertices_;
	double ratio_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrator");
	CropperCalibrator calibrator_object;

	ros::Rate rate(30);
	while (ros::ok() && !calibrator_object.cropped)
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
