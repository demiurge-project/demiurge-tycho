/* http://wiki.ros.org/image_transport/Tutorials/PublishingImages */

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class Cropper
{
public:
	Cropper()
		: image_transport_(node_handle_), mask_defined(false)
	{
		image_sub_ = image_transport_.subscribe("camera/image_rect_color", 1,
		                                        &Cropper::imageCallback, this);
		image_pub_ = image_transport_.advertise("image_cropped", 1);
		
		// Retrieve vertices from YAML
		std::vector<int> point;
		int i = 0;
		while (true)
		{
			std::string param_name = "cropper/point_" + std::to_string(i);

			if (!node_handle_.getParam(param_name, point))
			{
				break;
			}

			vertices_.push_back(cv::Point(point.at(0), point.at(1)));
			i++;
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

		// Generate mask from vertices
		if (!mask_defined)
		{
			// Mask is black with white where our ROI is
			mask_ = cv::Mat::zeros(img_.rows, img_.cols, CV_8UC1);
			std::vector<std::vector<cv::Point>> points{vertices_};
			cv::fillPoly(mask_, points, cv::Scalar(255, 255, 255));
		}

		// Apply mask to image
		img_.copyTo(roi_, mask_);

		// Send image
		out_msg_ = cv_bridge::CvImage(std_msgs::Header(msg->header), "bgr8", roi_).toImageMsg();
		image_pub_.publish(out_msg_);
	}

private:
	ros::NodeHandle node_handle_;
	image_transport::ImageTransport image_transport_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	sensor_msgs::ImagePtr out_msg_;
	cv::Mat img_, roi_, mask_;
	bool mask_defined;
	std::vector<cv::Point> vertices_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cropper");
	Cropper cropper_object;
	ros::spin();
	return 0;
}
