#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <math.h>

class SubscribeAndPublish
{
public:

    struct params{
        double period;
    } config;

    struct transformerData{
        int count;
        std::map<int, ros::Publisher> list_publishers_;
        std::map<int, nav_msgs::Odometry> list_messages_;
        std::string namespace_;
    } transformer;
 
    SubscribeAndPublish() 
    {
        ros::NodeHandle nhParam("~");
 
        nhParam.param("period", config.period, 0.1);

        // Empty pose mesage
        nav_msgs::Odometry robot_pose_msg;
 
        //Topic you want to publish
        for (int i = 0; i < 30; i++){
            // Create topic name
            char numstr[21];
            std::string topic_prefix = "epuck_";
            sprintf(numstr, "%d", i);
            std::string topic_name = topic_prefix + numstr + "/odom";

            // Add publisher to list of publishers
            pub_odom_ = nodeHandle.advertise<nav_msgs::Odometry>(topic_name, 1);
            transformer.list_publishers_.insert(std::pair<int, ros::Publisher>(i, pub_odom_));
            transformer.list_messages_.insert(std::pair<int, nav_msgs::Odometry>(i, robot_pose_msg));
        }
 
        //Topic you want to subscribe
        sub_tracker_ = nodeHandle.subscribe("tracker/positions", 20, &SubscribeAndPublish::trackingCallback, this);

        //Timer
        timer = nodeHandle.createTimer(ros::Duration(config.period), &SubscribeAndPublish::timerCallback, this);

        // Get namespace
        transformer.namespace_ = ros::this_node::getNamespace();
        std::cout << "Namespace: " << transformer.namespace_ << std::endl;
    }
 
    void timerCallback(const ros::TimerEvent&)
    {
        pub_markers();

        return;
    }

    void trackingCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
    {
        int size = msg->data.size(); //Get size of incoming list

        if (size != 0)
        {
            for (int i = 1; i < size; i += 9)
            { // For each marker there is 9 integers of information: 1 for the id and 2 for each corner coordinate

                int robot_id = msg->data.at(i);
                        
                // Compute center for each robot given the coordinates of the 4 marker corners
                int x1 = msg->data.at(1 + i);
                int x2 = msg->data.at(3 + i);
                int x3 = msg->data.at(5 + i);
                int x4 = msg->data.at(7 + i);

                int center_x = (x1 + x2 + x3 + x4) / 4;

                int y1 = msg->data.at(2 + i);
                int y2 = msg->data.at(4 + i);
                int y3 = msg->data.at(6 + i);
                int y4 = msg->data.at(8 + i);

                int center_y = (y1 + y2 + y3 + y4) / 4;

                // Conversion to ARGoS scenario, for Habanero experiments
                //double pose_x = (center_y - 28) * 0.001175; // TODO: check why inverted
                //double pose_y = -(center_x + 29) * 0.001175;

                // Transform as it comes from the tracker
                /*
                double pose_x = center_x; 
                double pose_y = center_y;
                double pose_yaw = atan2(y2-y1,x2-x1);
                */

                // Corrected transform to fit coordinate frame in the arena (see postit in the table)
                double pose_x = -center_y; 
                double pose_y = -center_x;
                double pose_yaw = atan2(x1-x2,y1-y2);
            
                // Create ROS message
                
                nav_msgs::Odometry robot_pose_msg;

                std::stringstream ss;
                ss << "epuck_" << robot_id;         
                //robot_pose_msg.header.frame_id = ss.str();
                robot_pose_msg.header.frame_id = "odom";
                ss.str("");
                ss << "base_link_" << robot_id;
                robot_pose_msg.child_frame_id = ss.str();
                robot_pose_msg.header.stamp = ros::Time::now();

                // Covariance matrix to account for systematic uncertainties
                // Values are calculated as the square of the size of a pixel projected on the arena floor (for x and y) and
                // the angular displacement of the diagonal of a marker when one corner is displaced by one pixel
                // Ignore z, roll and pitch for now
                // These values are provisional. Proper variance values should be obtained experimentally
                robot_pose_msg.pose.covariance = {4e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 4e-6, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 1.6e-3};

                robot_pose_msg.pose.pose.position.x = pose_x * 0.001;
                robot_pose_msg.pose.pose.position.y = pose_y * 0.001;
                robot_pose_msg.pose.pose.position.z = 0;

                tf2::Quaternion orientation_quat;
                tf2::Quaternion offset_quat;

                orientation_quat.setRPY(0, 0, pose_yaw);
                    
                /* Adjusting transform - Hack to correct offset 
                    This correction must be calibrated at the height of the robot,
                    otherwise there will be distortion on position of the pixels.
                    Probably, the distortion will couse the error to grow toward the
                    edges.
                */

                if (transformer.namespace_ == "/camera_0"){
                    robot_pose_msg.pose.pose.position.x = (pose_x + 43) * 0.001;
                    robot_pose_msg.pose.pose.position.y = (pose_y + 646) * 0.001;
                    offset_quat.setRPY(0, 0, 0);
                    orientation_quat = offset_quat * orientation_quat;
                    orientation_quat.normalize();
                    tf2::convert(orientation_quat, robot_pose_msg.pose.pose.orientation);
                }
                else if (transformer.namespace_ == "/camera_1"){
                    robot_pose_msg.pose.pose.position.x = (pose_x + 65) * 0.001;
                    robot_pose_msg.pose.pose.position.y = (pose_y - 572) * 0.001;
                    offset_quat.setRPY(0, 0, -0.0499584);
                    orientation_quat = offset_quat * orientation_quat;
                    orientation_quat.normalize();
                    tf2::convert(orientation_quat, robot_pose_msg.pose.pose.orientation);
                }
                else if (transformer.namespace_ == "/camera_2"){
                    robot_pose_msg.pose.pose.position.x = (pose_x + 8)  * 0.001;
                    robot_pose_msg.pose.pose.position.y = (pose_y - 46) * 0.001;
                    offset_quat.setRPY(0, 0, -0.0475831);
                    orientation_quat = offset_quat * orientation_quat;
                    orientation_quat.normalize();
                    tf2::convert(orientation_quat, robot_pose_msg.pose.pose.orientation);
                }

                // Publish to corresponding topic
                transformer.list_messages_.at(robot_id) = robot_pose_msg;  
            }
        }
    }
 
    void pub_markers()
    {
        for (int i = 0; i < 30; i++)
        {
            transformer.list_publishers_.at(i).publish(transformer.list_messages_.at(i));

            /*
                TODO: UKF might require that this transform is published,
                however, this might be computationally expensive here.
                Maybe it is a better ideat to separate it in a new timer, 
                however topics migh not be sync anymore.
            */
            /*
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            
            transformStamped.header.stamp = transformer.list_messages_.at(i).header.stamp;
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = turtle_name;
            transformStamped.transform.translation.x = msg->x;
            transformStamped.transform.translation.y = msg->y;
            transformStamped.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, msg->theta);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            br.sendTransform(transformStamped);
            */
        }

        return;
    }
    
private:
    ros::Publisher pub_odom_;
    ros::Subscriber sub_tracker_;
    ros::Timer timer;
    ros::NodeHandle nodeHandle;
 
};//End of class SubscribeAndPublish
 
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "tracker");
    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;
 
    ROS_INFO("Transformer Node Ok!!!");
 
    ros::spin();
 
    return 0;
}
