
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;





int main(int argc, char** argv)
{
    Mat img;
    // Initalize ros publishing
    ros::init(argc, argv, "image_grabber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub = it.advertise("/image_raw", 1);

    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(30);

   //Read video from file
   VideoCapture cap("src/grabber/videos/IMG_0440.MOV"); // test video include in videos folder

   // Check it loaded
   if(!cap.isOpened()){
       cout << "Error opening video stream or file" << endl;
       return -1;
     }

   while(nh.ok()){

       // Capture frame-by-frame
       cap >> img;
       Mat imgcpy;
       img.copyTo(imgcpy);

       // If the frame is empty, break immediately
       if (img.empty())
         break;

       // Convert to ros message
       msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgcpy).toImageMsg();
       pub.publish(msg);

       ros::spinOnce();
       loop_rate.sleep();

       // Press  ESC on keyboard to exit
       char c=(char)waitKey(25);
       if(c==27)
         break;
     }



   //cap.release();


   // Show results
   //namedWindow("Result",1);
   //imshow("Result",ROI);
   //waitKey(5000);
}