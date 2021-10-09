/* This node is used to publish sudo obstacle that is static in a shape 
 * of a point :(( HMMM, yeah I know it is simple *rolling eyes
 * a polygon http://docs.ros.org/en/api/costmap_converter/html/msg/ObstacleMsg.html 
 */

#include <ros/ros.h>
#include <thread>
#include <math.h>

#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <geometry_msgs/Point32.h>

#include <memory>

using namespace std;


int main (int argc, char** argv)
{
    ros::init(argc, argv, "test_obstacle_msg");
    ros::NodeHandle nh;
    
    ros::Publisher obstacle_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/test_optim_node/obstacles", 10);
    
    ros::Rate rate(10.0);
    
    double t = 0.0;

    //void publishObstacle(); 

    while (ros::ok())
    {
        costmap_converter::ObstacleArrayMsg obstacle_msg;
        obstacle_msg.header.stamp = ros::Time::now();
        obstacle_msg.header.frame_id = "odom";

        geometry_msgs::Point32 point_obs;
        point_obs.x = 1.5;
        point_obs.y = sin(t);
        point_obs.z = 0;

        unique_ptr<costmap_converter::ObstacleMsg> ObstaclePtr = make_unique <costmap_converter::ObstacleMsg> ();
        ObstaclePtr->polygon.points.emplace_back(point_obs);

        obstacle_msg.obstacles.emplace_back(*ObstaclePtr);
        obstacle_msg.obstacles.end()->id = 0;

        t += 0.1;
        obstacle_pub.publish(obstacle_msg);
        rate.sleep();
    }

    ros::spin();

    return 0;
    
}
