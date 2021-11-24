#include "custom_blocks.h"

ros::Publisher* Algorithm::vizPub;
tf::TransformListener* Algorithm::tfListener;
tf::StampedTransform Algorithm::laserTf;

string block_custom::WallFollow::generateDebugName()
{
    return "WallFollow";
}

block_custom::WallFollow::WallFollow(): block::SequenceBlock()
{
    Algorithm algorithm;
    shared_ptr<Sequence> sequence = make_shared<Sequence>();
    sequence->addVariable<geometry_msgs::Twist>("twist");
    sequence->compose
    (
        make_shared<block_ros::Subscribe<sensor_msgs::LaserScan>>("scan", 1, [&](const sensor_msgs::LaserScan msg)
        {
            geometry_msgs::Twist tw;
            algorithm.wallFollower(msg,Algorithm::RIGHT,1.0, 0.5, tw);
            geometry_msgs::Twist& twv = Sequence::getVariable<geometry_msgs::Twist>("twist");
            twv.angular.z = tw.angular.z;
            twv.linear.x = tw.linear.x;
            return true;
        }),
        make_shared<block::Function>([&]
        {
            geometry_msgs::Twist& twv = Sequence::getVariable<geometry_msgs::Twist>("twist");
            Sequence::printDebug("linear:"+ to_string(twv.linear.x));
            Sequence::printDebug("angular"+to_string(twv.angular.z));
        }),
        make_shared<block_ros::Publish<geometry_msgs::Twist>>(&(sequence->getSequenceVariable<geometry_msgs::Twist>("twist")),"cmd_vel",1)
    );
    this->setSequence(sequence);
}
