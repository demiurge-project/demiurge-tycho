#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class Tracker
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub;
    cv::Mat camera_matrix, dist_coeff; // These can be set after camera calibration to undistort image
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

public:
    Tracker()
        : it_(nh_)
    {
        // Subscribe to camera
        bool crop;
        nh_.getParam("tracker/crop", crop);
        if (crop)
        {
            image_sub_ = it_.subscribe("image_cropped", 20, &Tracker::imageCb, this);
        }
        else
        {
            image_sub_ = it_.subscribe("camera/image_rect_color", 20, &Tracker::imageCb, this);
        }
        
        // Advertise a list of integers
        pub = nh_.advertise<std_msgs::UInt32MultiArray>("tracker/positions_stamped", 20);

        // Set aruco dictionary to use for marker detection
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        parameters = cv::aruco::DetectorParameters::create();
    }

    // Subscriber callback function
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

        // Get time stamp
        ros::Time stamp = cv_ptr->header.stamp;

        // List that will store detection results
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;

        // Aruco detection function
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, parameters);

        // Add time stamp and all marker positions to list
        // std_msgs::UInt32MultiArray has no header, so stamp is introduced in the first two entries: (sec, nsec)
        std_msgs::UInt32MultiArray positions_stamped;
        positions_stamped.data.push_back(stamp.sec);
        positions_stamped.data.push_back(stamp.nsec);
        for (int i = 0; i < corners.size(); i++)
        {
            positions_stamped.data.push_back(ids.at(i));
            for (int j = 0; j < corners.at(i).size(); j++)
            {
                positions_stamped.data.push_back(corners.at(i).at(j).x);
                positions_stamped.data.push_back(corners.at(i).at(j).y);
            }
        }
        // Publish
        pub.publish(positions_stamped);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker");
    Tracker tracker_object;
    ros::spin();
    return 0;
}
