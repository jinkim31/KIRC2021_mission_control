#include <ros/ros.h>
#include <sequence_ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <lz4frame.h>
#include "../Algorithm/algorithm.h"
#include "../Algorithm/custom_blocks.h"

using namespace std;
using namespace seq;
using namespace ld2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    geometry_msgs::Twist twist;
    sensor_msgs::LaserScan scan;

    Algorithm::init();

    bool a;
    Sequence sequenceMission5
    (
        "mission 3",
        make_shared<block::Delay>(0.5),
        make_shared<block_ros::ServiceCall<std_srvs::Empty>>("move_base/clear_costmaps"),
        make_shared<block_ros::NavGoal>(4.58, 2.26, 0, 0.0, 0.0, -0.71, 0.71),
        make_shared<block_ros::WaitForNavReachLatched>(),
        make_shared<block::Delay>(3.0),
        make_shared<block_ros::NavGoal>(7.70, 0.88, 0.0, 0.0, 0.0, 0.0, 1.0),
        make_shared<block_ros::WaitForNavReachLatched>(),
        make_shared<block::LoopSequence>(make_shared<LambdaCondition>([&]{return a;}), make_shared<Sequence>
        (
            "wallfollow",
            make_shared<block_ros::Subscribe<sensor_msgs::LaserScan>>("scan", 1, [&](const sensor_msgs::LaserScan msg)
            {
                static int cnt = 0;
                Sequence::printDebug("callback " + to_string(cnt++));
                scan = msg;
                return true;
            }),
            make_shared<block::Function>([&]
            {
                if(scan.ranges[M_PI/2/scan.angle_increment]>1.0)
                {
                    a = true;
                }
                Algorithm::wallFollower(scan, Algorithm::WallToFollow::RIGHT, 0.8, 0.1, twist);
            }),
            make_shared<block_ros::Publish<geometry_msgs::Twist>>(&twist,"cmd_vel",1)
        )),
        make_shared<block_ros::Twist>(0,0)
    );
    sequenceMission5.compile(true);

    /*
     * MISSION 3
     */

    Sequence sequenceMission3;
    sequenceMission3.addVariable<sensor_msgs::LaserScan>("scan");
    sequenceMission3.addVariable<bool>("isObstacle", false);
    sequenceMission3.addVariable<double>("angleStart");
    sequenceMission3.addVariable<double>("angleEnd");
    sequenceMission3.addVariable<double>("distanceMean");
    sequenceMission3.compose
    (
        "mission 3",
        make_shared<block::LoopSequence>
        (make_shared<LambdaCondition>([&]{return
        Sequence::getVariable<bool>("isObstacle") &&
        Sequence::getVariable<double>("distanceMean") < 1.0;}
        ), make_shared<Sequence>
        (
            "wallfollow & obstacle",
            make_shared<block_custom::WallFollow>(),
            make_shared<block_ros::Subscribe<sensor_msgs::LaserScan>>("scan", 1, [&](const sensor_msgs::LaserScan msg)
            {
                Sequence::getVariable<sensor_msgs::LaserScan>("scan")=msg;
                return true;
            }),
            make_shared<block::Function>([&]
            {
                double angleStart, angleEnd;
                double distanceMean;
                bool isObstacle = Algorithm::findObstacle
                (Sequence::getVariable<sensor_msgs::LaserScan>("scan"),
                angleStart,
                angleEnd, distanceMean);
                Sequence::getVariable<double>("angleStart")=angleStart;
                Sequence::getVariable<double>("angleEnd")=angleEnd;
                Sequence::getVariable<double>("distanceMean")=distanceMean;
                Sequence::getVariable<bool>("isObstacle")=isObstacle;
            })
        )),
        make_shared<block::Debug>("Obstacle Detected"),
        make_shared<block_ros::Twist>(0.0, 0.0),
        make_shared<block::LoopSequence>
        (make_shared<LambdaCondition>([&]{return !Sequence::getVariable<bool>("isObstacle");}),make_shared<Sequence>
        (
            "obstacle removal",
            make_shared<block::Debug>("!Manipulation Here!"),
            make_shared<block::Function>([&]
            {
                double angleStart, angleEnd;
                double distanceMean;
                bool isObstacle = Algorithm::findObstacle
                (Sequence::getVariable<sensor_msgs::LaserScan>("scan"),
                angleStart,
                angleEnd, distanceMean);
                Sequence::getVariable<bool>("isObstacle")=isObstacle;
            })
        )),
        make_shared<block::Debug>("Obstacle Removed"),
        make_shared<block::LoopSequence>
        (make_shared<LambdaCondition>([&]{return false;}),make_shared<Sequence>
        (
            "door approach",
            make_shared<block_custom::WallFollow>()
        ))
    );

    sequenceMission3.compile(true);
    sequenceMission3.start();

    while (ros::ok())
    {
        ros::spinOnce();
        Sequence::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
