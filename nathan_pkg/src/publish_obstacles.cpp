/* Practicing Costmap converter node
 * Writing C++ node for the optimized obstacles and dynamic obstacles
 *
 * Point obstacle: provide a polygon with a single vertex
 * Circular obstacle: provide a polygon with a single vertex and a non-zero radius value.
 * Line obstacle: provide a polygon with two vertices
 * Polygon obstacle: provide a polygon with more than 2 vertices;
 * the polygon will be considered to be closed between the last and the first vertex.
 * https://github.com/rst-tu-dortmund/costmap_converter/tree/master/src
 */


#include <ros/ros.h>
//obstacle message
#include <costmap_converter/ObstableMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <boost/thread.hpp> // for smart pointers

// #include <costmap_converter/costmap_converter_interface.h>

//geometry message
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>


namespace costmap_converter
{

typedef boost::shared_ptr<costmap_converter::ObstacleMsg> ObstaclePtr;

class CustomObstacles
{
public:
    CustomObstacles(); //Constructors
    ~CustomObstacles(); //Destructors

private:
    ros::NodeHandle nh;
    ros::Publisher obs_pub;

    void publishObstacles();

}; // class

CustomObstacles::CustomObstacles()
{
    //topic has to be correct for the test_optim_node and listen to correctly
    obs_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/test_optim_node/obstacles", 1);
}

CustomObstacles::publishObstacles() 
{
    costmap_converter::ObstacleArrayMsg obstacles_msg; 
    // SO we can treat obstacles_msg.obstacles as a vector.
    int ID;

    obstacles_msg.header.stamp = ros::Time::now();
    obstacles_msg.header.frame_id = "odom";

    //create individual obstacles to put into the array
    //costmap_converter::ObstacleMsg individual_obs;
    if (obstacles_msg.obstacles.empty())
    {
        ID = 0;
    }

    //Make this to delete the shared pointer when it escapes the block of code
    {
        ObstaclePtr MakePtr = make_shared<costmap_converter::ObstacleMsg>();

        geometry_msgs::Point32 point_msg;
        point_msg.x = 1.5;
        point_msg.y = 0;
        point_msg.z = 0;

        MakePtr->id = ID;
        MakePtr->polygon.points.emplace_back(point_msg);

        obstacle_msg.obstacle.emplace_back(*MakePtr);
        ID++;
        
    } //Add a point obstacle


    

   
    
    
    //a polygon http://docs.ros.org/en/api/costmap_converter/html/msg/ObstacleMsg.html 
    
    
}

} //namespace

int main (int argc, char** argv)
{
    ros::init(argc, argv, "test_obstacle_msg");
    costmap_converter::CustomObstacles Obstacle_obj;

    ros::spin();

    return 0;
    
}
