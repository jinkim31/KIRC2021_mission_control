#ifndef MISSION_CONTROL_ALGORITHM_H
#define MISSION_CONTROL_ALGORITHM_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "../GRANSAC/include/GRANSAC.hpp"
#include "../GRANSAC/examples/LineModel.hpp"
#include "../LidarProcessing2D/lidar_2d_processing.h"
#include "../Algorithm/custom_blocks.h"

#define PI 3.1415926535

using namespace std;
using namespace ld2;

class Algorithm
{
private:
    static ros::Publisher* vizPub;
    static tf::TransformListener *tfListener;
    static tf::StampedTransform laserTf;

public:
    static void init();
    enum WallToFollow
    {
        LEFT,
        RIGHT
    };
    static bool wallFollower(const sensor_msgs::LaserScan & scan, WallToFollow wallToFollow, double clearance, double vel, geometry_msgs::Twist& result);
    static bool findObstacle(const sensor_msgs::LaserScan & msg, double &angleStart, double &angleEnd, double& distanceMean);
};


#endif