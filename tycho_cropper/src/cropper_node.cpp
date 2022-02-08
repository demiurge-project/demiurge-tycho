
/*
https://stackoverflow.com/questions/42190687/opencv-how-can-i-select-a-region-of-image-irregularly-with-mouse-event-c-c

http://wiki.ros.org/image_transport/Tutorials/PublishingImages
*/


#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;



class Cropper
{
    // cropper algorithm variables
    Mat img,ROI,mask;
    vector<Point> vertices;
    bool cropped = false;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    sensor_msgs::ImagePtr out_msg;


public:
    Cropper()
        : it_(nh_){

        image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &Cropper::imageCb, this);
        image_pub_ = it_.advertise("/image_cropped", 1);
    }

    // mouse callback function, has to be static
    static void
    CallBackFunc(int event,int x,int y,int flags,void* userdata)
    {
        // Should perhaps check if userdata is not nullptr
        Cropper* cropper = reinterpret_cast<Cropper*>(userdata);
        cropper->CallBackFunc(event, x, y); // Calls class callback function, which is not static

    }


    void
    CallBackFunc(int event,int x,int y)
    {
       if(event==EVENT_RBUTTONDOWN){
          cout << "Right mouse button clicked at (" << x << ", " << y << ")" << endl;
          if(vertices.size()<2){
             cout << "You need a minimum of three points!" << endl;
             return;
          }

          // Close polygon
          line(img,vertices[vertices.size()-1],vertices[0],Scalar(0,0,0));

          // Mask is black with white where our ROI is
          mask= Mat::zeros(img.rows,img.cols,CV_8UC1);
          vector<vector<Point>> pts{vertices};
          fillPoly(mask,pts,Scalar(255,255,255));
          img.copyTo(ROI,mask);


          cropped=true;

          return;
       }
       if(event==EVENT_LBUTTONDOWN){
          cout << "Left mouse button clicked at (" << x << ", " << y << ")" << endl;
          if(vertices.size()==0){
             // First click - just draw point
             img.at<Vec3b>(x,y)=Vec3b(255,0,0);
          } else {
             // Second, or later click, draw line to previous vertex
             line(img,Point(x,y),vertices[vertices.size()-1],Scalar(0,0,0));
          }
          vertices.push_back(Point(x,y));
          return;
       }
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

        // Store image for use in cropping algorithm
        img = cv_ptr->image;

        // If polygon has not been delimited start cropping process
        if(!cropped){

                //Create a window
                namedWindow("ImageDisplay",1);

                // Register a mouse callback
                setMouseCallback("ImageDisplay",CallBackFunc,this);

                // Cropped is false until a closed polygon has been defined
                while(!cropped){
                    imshow("ImageDisplay",img);
                    waitKey(50);
                }

                destroyWindow("ImageDisplay");
            }

        // Apply mask to image
        img.copyTo(ROI,mask);

        // Send image
        out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ROI).toImageMsg();
        image_pub_.publish(out_msg);

    }


};

int main(int argc, char** argv)
{

    // Initalize ros publishing
    ros::init(argc, argv, "crpr");
    Cropper crpr;
    ros::spin();
    return 0;

}
