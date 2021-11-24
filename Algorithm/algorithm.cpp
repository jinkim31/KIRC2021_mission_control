#include "algorithm.h"

void Algorithm::init()
{
    cout<<"init start"<<endl;
    ros::NodeHandle nh;
    vizPub = new ros::Publisher;
    *vizPub = nh.advertise<visualization_msgs::Marker>("wallFollowViz", 1);
    cout<<"init"<<endl;
//
//    try
//    {
//        tfListener.lookupTransform("base_link", "base_scan", ros::Time::now(), laserTf);
//    }catch(tf::TransformException e)
//    {
//        ROS_ERROR(e.what());
//    }
}

bool Algorithm::wallFollower(const sensor_msgs::LaserScan &scan, Algorithm::WallToFollow wallToFollow, double clearance, double vel, geometry_msgs::Twist &result)
{
    /*
     * Visualization marker init
     */
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = "/laser";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "wallFollower";
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.lifetime = ros::Duration(0.5);
    pointMarker.id = 0;
    pointMarker.type = visualization_msgs::Marker::POINTS;
    pointMarker.scale.x = 0.02;
    pointMarker.scale.y = 0.02;
    pointMarker.scale.z = 0.02;
    pointMarker.color.r = 0.0;
    pointMarker.color.g = 1.0;
    pointMarker.color.b = 0.3;
    pointMarker.color.a = 1.0;

    visualization_msgs::Marker ransacMarker;
    ransacMarker.header.frame_id = "/laser";
    ransacMarker.header.stamp = ros::Time::now();
    ransacMarker.ns = "wallFollower";
    ransacMarker.action = visualization_msgs::Marker::ADD;
    ransacMarker.pose.orientation.w = 1.0;
    ransacMarker.lifetime = ros::Duration(0.5);
    ransacMarker.id = 1;
    ransacMarker.type = visualization_msgs::Marker::LINE_STRIP;
    ransacMarker.scale.x = 0.02;
    ransacMarker.scale.y = 0.02;
    ransacMarker.scale.z = 0.02;
    ransacMarker.color.r = 0.0;
    ransacMarker.color.g = 1.0;
    ransacMarker.color.b = 0.3;
    ransacMarker.color.a = 1.0;

    visualization_msgs::Marker goalMarker;
    goalMarker.header.frame_id = "/laser";
    goalMarker.header.stamp = ros::Time::now();
    goalMarker.ns = "wallFollower";
    goalMarker.action = visualization_msgs::Marker::ADD;
    goalMarker.pose.orientation.w = 1.0;
    goalMarker.lifetime = ros::Duration(0.5);
    goalMarker.id = 2;
    goalMarker.type = visualization_msgs::Marker::SPHERE;
    goalMarker.scale.x = 0.1;
    goalMarker.scale.y = 0.1;
    goalMarker.scale.z = 0.1;
    goalMarker.color.r = 1.0;
    goalMarker.color.g = 0.4;
    goalMarker.color.b = 0.3;
    goalMarker.color.a = 1.0;

    /*
     * TF & wall filter
     */
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> points;

    unsigned int nScanData = (scan.angle_max - scan.angle_min)/scan.angle_increment;
    double angle;
    int i;

    for(i=0; i < nScanData; i++)
    {
        angle = scan.angle_min + scan.angle_increment*i;
        if(angle<0) angle+=2*PI;
        if(wallToFollow == LEFT && (PI<angle&&angle<2*PI)) continue;
        if(wallToFollow == RIGHT && (0<angle&&angle<PI)) continue;

        geometry_msgs::Point p;
        p.x = scan.ranges[i] * cos(angle);
        p.y = scan.ranges[i] * sin(angle);
        p.z = 0;

        if(isnan(p.x) || isnan(p.y) || isinf(p.x) || isinf(p.y)) continue;
        pointMarker.points.push_back(p);

        std::shared_ptr<GRANSAC::AbstractParameter> pointPtr = std::make_shared<Point2D>(p.x, p.y);
        points.push_back(pointPtr);
    }

    //vizPub->publish(pointMarker);

    /*
     * Voxel filter
     */

    /*
     * RANSAC
     */
    GRANSAC::RANSAC<Line2DModel, 2> estimator;
    estimator.Initialize(0.1, 100);
    estimator.Estimate(points);

    auto BestLine = estimator.GetBestModel();
    if (!BestLine) return false;
    auto BestLinePt1 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[0]);
    auto BestLinePt2 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[1]);
    if (!BestLinePt1 || !BestLinePt2) return false;

    geometry_msgs::Point p1, p2;
    p1.x = BestLinePt1->m_Point2D[0];
    p1.y = BestLinePt1->m_Point2D[1];
    p1.z = 0;
    p2.x = BestLinePt2->m_Point2D[0];
    p2.y = BestLinePt2->m_Point2D[1];
    p2.z = 0;

    ransacMarker.points.push_back(p1);
    ransacMarker.points.push_back(p2);

    vizPub->publish(ransacMarker);

    /*
     * Goal generation
     */

    double waypointDistance = 1.0;

    tf::Vector3 wallDir = tf::Vector3(p2.x - p1.x, p2.y - p1.y, 0).normalize();
    if(wallDir.x() < 0)
    {
        wallDir = wallDir.rotate(tf::Vector3(0,0,1.0), PI);
    }
    tf::Vector3 pointOnWallToRobot(0-p1.x, 0-p1.y, 0);
    double distance = wallDir.cross(pointOnWallToRobot).length() / wallDir.length();
    tf::Vector3 normalToWall;
    if(wallToFollow==RIGHT) normalToWall = wallDir.rotate(tf::Vector3(0,0,1.0), PI/2).normalize();
    else if(wallToFollow==LEFT) normalToWall = wallDir.rotate(tf::Vector3(0,0,1.0), -PI/2).normalize();
    tf::Vector3 goal = wallDir * waypointDistance + (clearance - distance) * normalToWall;

    goalMarker.pose.position.x = goal.x();
    goalMarker.pose.position.y = goal.y();
    //vizPub.publish(goalMarker);

    /*
     * Pure pursuit
     */
    double lookaheadDistance = goal.length();
    double radius = lookaheadDistance*lookaheadDistance/(2*goal.y());

    /*
     * Twist generation;
     */

    double linearVel = vel;
    result.linear.x = linearVel;
    result.angular.z = linearVel/radius;

    return true;
}

bool Algorithm::findObstacle(const sensor_msgs::LaserScan & msg, double &angleStart, double &angleEnd, double& distanceMean)
{
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = "/laser";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "wallFollower";
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.lifetime = ros::Duration(0.5);
    pointMarker.id = 3;
    pointMarker.type = visualization_msgs::Marker::POINTS;
    pointMarker.scale.x = 0.04;
    pointMarker.scale.y = 0.04;
    pointMarker.scale.z = 0.04;
    pointMarker.color.r = 1.0;
    pointMarker.color.g = 0.2;
    pointMarker.color.b = 0.1;
    pointMarker.color.a = 1.0;

    Ranges filtered;
    hysteresisFilter(msg.ranges, filtered, 0.05);
    Ranges mask;
    generateGaussianMask(mask, 0.5);
    Ranges gaussian;
    convolute(filtered, mask, gaussian);
    // laplacian
    Ranges derivative, laplacian;
    diff(gaussian, derivative);
    diff(derivative, laplacian);
    //zero crossing
    Ranges zeroCrossing;
    findZeroCrossing(laplacian, zeroCrossing);
    //printRanges(msg.ranges);
    //printRanges(gaussian);
    //printRanges(laplacian);
    //printRanges(zeroCrossing);

    //split
    vector<Subranges>splitRange;
    ld2::splitRanges(filtered, zeroCrossing, splitRange, 2);

    vector<int> excludeIndexList;
    excludeIndexList.push_back(1080*(1.0/6));//right
    excludeIndexList.push_back(1080*(5.0/6));//left
    int startIdx, len;
    bool result =  assessForObstacle(splitRange, 1.5, excludeIndexList, msg.ranges.size()/2, startIdx, len);

    if(result)
    {
        double distanceSum = 0;
        for(int i=startIdx; i<startIdx + len; i++)
        {
            distanceSum += msg.ranges[i];
            geometry_msgs::Point p;
            p.x = filtered[i] * cos(msg.angle_min + msg.angle_increment*(i));
            p.y = filtered[i] * sin(msg.angle_min + msg.angle_increment*(i));
            pointMarker.points.push_back(p);
        }
        distanceMean = distanceSum / len;
        angleStart = msg.angle_min+msg.angle_increment*startIdx;
        angleEnd = msg.angle_min+msg.angle_increment*(startIdx+len);

        vizPub->publish(pointMarker);
    }

    return result;
}
