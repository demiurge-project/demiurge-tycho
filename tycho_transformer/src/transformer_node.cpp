#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class SubscribeAndPublish
{
public:
    struct transformerData
    {
        std::map<int, ros::Publisher> list_publishers;
        std::map<int, nav_msgs::Odometry> list_messages;
        std::string ns;
        double x_offset;
        double y_offset;
        double angular_offset;
        double scale_factor;
    } transformer;
 
    SubscribeAndPublish() 
    {
        // Empty pose mesage
        nav_msgs::Odometry robot_pose_msg;
 
        //Topic you want to publish
        for (int i = 0; i < 50; i++)
        {
            // Create topic name
            char numstr[21];
            std::string topic_prefix = "epuck_";
            sprintf(numstr, "%d", i);
            std::string topic_name = topic_prefix + numstr + "/odom";

            // Add publisher to list of publishers
            pub_odom_ = node_handle_.advertise<nav_msgs::Odometry>(topic_name, 20);
            transformer.list_publishers.insert(std::pair<int, ros::Publisher>(i, pub_odom_));
            transformer.list_messages.insert(std::pair<int, nav_msgs::Odometry>(i, robot_pose_msg));
        }
 
        //Topic you want to subscribe
        sub_tracker_ = node_handle_.subscribe("tracker/positions_stamped", 20, &SubscribeAndPublish::transformerCallback, this);

        // Get namespace
        transformer.ns = ros::this_node::getNamespace();

        // Get config data
        std::string param = transformer.ns + "/transformer/x_offset";
        node_handle_.getParam(param, transformer.x_offset);
        param = transformer.ns + "/transformer/y_offset";
        node_handle_.getParam(param, transformer.y_offset);
        param = transformer.ns + "/transformer/angular_offset";
        node_handle_.getParam(param, transformer.angular_offset);
        param = transformer.ns + "/transformer/scale_factor";
        node_handle_.getParam(param, transformer.scale_factor);
    }

    void transformerCallback(const std_msgs::UInt32MultiArrayConstPtr& msg)
    {
        int size = msg->data.size(); // Get size of incoming list

        if (size > 2) // Timestamp is always in the first two entires
        {
            for (int i = 2; i < size; i += 9) // For each marker there is 9 integers of information: 1 for the id and 2 for each corner coordinate
            { 
                // Create ROS message
                nav_msgs::Odometry robot_pose_msg;

                // Set header
                int robot_id = msg->data.at(i);

                std::stringstream ss;
                ss << "epuck_" << robot_id;
                robot_pose_msg.header.frame_id = "odom";
                ss.str("");
                ss << "base_link_" << robot_id;
                robot_pose_msg.child_frame_id = ss.str();
                robot_pose_msg.header.stamp = ros::Time(msg->data.at(0), msg->data.at(1));

                // Transform (translate + rotate + scale) every marker corner to the arena frame
                std::vector<double> x(4);
                std::vector<double> y(4);
                for (int j = 0; j < 4; ++j)
                {
                    // Transform left-handed (camera) frame to right-handed (arena) frame
                    int x_tracker = -msg->data.at(2*j + 1 + i);
                    int y_tracker = msg->data.at(2*j + 2 + i);

                    // Translate and scale
                    double x_tmp = (x_tracker - transformer.x_offset) * transformer.scale_factor;
                    double y_tmp = (y_tracker - transformer.y_offset) * transformer.scale_factor;

                    // Rotate
                    x.at(j) = x_tmp*cos(-transformer.angular_offset) - y_tmp*sin(-transformer.angular_offset);
                    y.at(j) = x_tmp*sin(-transformer.angular_offset) + y_tmp*cos(-transformer.angular_offset);
                }

                // Compute position of the center of the robot marker
                double pose_x = 0.0;
                double pose_y = 0.0;
                for (int j = 0; j < 4; ++j)
                {
                    pose_x += x.at(j) / 4.0;
                    pose_y += y.at(j) / 4.0;
                }

                robot_pose_msg.pose.pose.position.x = pose_x;
                robot_pose_msg.pose.pose.position.y = pose_y;
                robot_pose_msg.pose.pose.position.z = 0.0;

                // Compute orientation (yaw in 2D) of the robot marker
                double pose_yaw = atan2(y.at(1) - y.at(0), x.at(1) - x.at(0));

                tf2::Quaternion orientation_quat;
                orientation_quat.setRPY(0, 0, pose_yaw);
                tf2::convert(orientation_quat, robot_pose_msg.pose.pose.orientation);

                // Set covariance for the cameras
                robot_pose_msg.pose.covariance = {1e-6, 0.0,  0.0, 0.0, 0.0, 0.0,
                                                  0.0,  1e-6, 0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0,  0.0, 0.0, 0.0, 1e-6};

                // Publish to corresponding topic
                transformer.list_messages.at(robot_id) = robot_pose_msg;
            }

            pub_markers();
        }
    }

    void pub_markers()
    {
        for (int i = 0; i < 50; i++)
        {
            if (transformer.list_messages.at(i).header.stamp.sec != 0)
            {
                transformer.list_publishers.at(i).publish(transformer.list_messages.at(i));
                transformer.list_messages.at(i).header.stamp.sec = 0;
            }
        }

        return;
    }
    
private:
    ros::Publisher pub_odom_;
    ros::Subscriber sub_tracker_;
    ros::NodeHandle node_handle_;
};
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "transformer");
    SubscribeAndPublish SAPObject;
    ros::spin();
 
    return 0;
}
