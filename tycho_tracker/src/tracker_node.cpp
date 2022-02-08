#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";


class Tracker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher pub;
  cv::Mat camera_matrix, dist_coeff; // These can be set after camera calibration to undistort image
  cv::Ptr<aruco::DetectorParameters> parameters;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  int offset_x, offset_y;

public:
  Tracker()
    : it_(nh_)
  {

    // Subscribe to Cropper node
    //image_sub_ = it_.subscribe("camera/image_raw", 1, &Tracker::imageCb, this);
    image_sub_ = it_.subscribe("camera/image_rect_color", 20, &Tracker::imageCb, this);
    // Advertize a list of integers
    pub = nh_.advertise<std_msgs::Int32MultiArray>("tracker/positions", 1);

    // Set aruco dictionary to use for marker detection
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    parameters = aruco::DetectorParameters::create();

     /* Aruco detection parameters can be here, values here are the default ones.
     parameters->adaptiveThreshWinSizeMin = 3;
     parameters->adaptiveThreshWinSizeMax = 23;
     parameters->adaptiveThreshWinSizeStep = 10;
    */
    // cv::namedWindow(OPENCV_WINDOW); // Commented out DAGR
  }

  ~Tracker()
  {
    // cv::destroyWindow(OPENCV_WINDOW); // Commented out DAGR
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

    // Change referential so that 0 is the center of the image
    offset_x = cv_ptr->image.size().width /2;
    offset_y = cv_ptr->image.size().height /2;

    /*  Following line can be used to undistort and image in place  */
    //cv:undistort(cv_ptr->image, cv_ptr->image, camera_matrix, dist_coeff);

    // List that will store detection results
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;

    // Aruco detection function
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, parameters);

    // Draw markers and axes
    /*
    if (ids.size() > 0)
    {
        // Draw markers on image
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        //Following code can be uncommented to draw x,y,z axis on marker as well, if camera has been calibrated
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeff, rvecs, tvecs);
        for(int i=0; i<ids.size(); i++)
        {
          cv::aruco::drawAxis(imgcpy, camera_matrix, dist_coeff, rvecs[i], tvecs[i], 0.1);
        }
    }
    */

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);  // Commented out DAGR
    // cv::waitKey(1);  // Commented out DAGR

    // Add all marker positions to list, with modified referential
    std_msgs::Int32MultiArray positions;
    positions.data.push_back(ids.size());
    for (int i = 0; i < corners.size(); i++)
    {
      positions.data.push_back(ids.at(i));
      for (int j = 0; j < corners.at(i).size(); j++)
      {
        positions.data.push_back(corners.at(i).at(j).x - offset_x);
        positions.data.push_back(corners.at(i).at(j).y - offset_y);
      }
    }
    // Publish
    pub.publish(positions);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tckr");
  Tracker tckr;
  ros::spin();
  return 0;
}
