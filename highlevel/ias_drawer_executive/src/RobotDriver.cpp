/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Poses.h>

#include <sensor_msgs/point_cloud_conversion.h>

RobotDriver::RobotDriver()
{
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1);
    subScan_ = nh_.subscribe("base_scan", 10, &RobotDriver::scanCallback, this);
    projector_ = 0;
    weHaveScan = false;
}

RobotDriver *RobotDriver::getInstance()
{
    if (!instance)
        instance = new RobotDriver();
    return instance;
}

void RobotDriver::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if (!projector_)
        projector_ = new laser_geometry::LaserProjection();

    scan_mutex.lock();
    sensor_msgs::PointCloud scan_cloud;
    projector_->projectLaser(*scan_in,scan_cloud);
    int step = 1;
    for (size_t k = 0; k < scan_cloud.points.size(); k += step)
    {
        scanPoints[k/step][0] = scan_cloud.points[k].x;
        scanPoints[k/step][1] = scan_cloud.points[k].y;
        numScanPoints = k / step;
        //ROS_INFO("SCAN POINTS : %i %f %f",numScanPoints, scanPoints[k/step][0], scanPoints[k/step][1]);
    }
    scan_mutex.unlock();
    weHaveScan = true;
}

bool RobotDriver::checkCollision(float relativePose[])
{
    ros::Rate rate(10);
    while (!weHaveScan)
    {
        rate.sleep();
        ros::spinOnce();
    }
    scan_mutex.lock();
    //ROS_INFO("SCAN POINTS : %i",numScanPoints);
    bool good = true;
    float padding = 0.05;
    for (size_t k = 0; good && (k < numScanPoints); k += 1)
    {
        float x = scanPoints[k][0] + relativePose[0] + 0.275;
        float y = scanPoints[k][1] + relativePose[1];
        if ((x < .325 + padding) && (x > -.325 - padding) && (y < .325 + padding) && (y > -.325 - padding))
        {
            //ROS_INFO("POINT %f %f",  scanPoints[k][0] , scanPoints[k][1]);
            good = false;
        }
    }
    scan_mutex.unlock();
    return good;
}

void RobotDriver::getRobotPose(tf::Stamped<tf::Pose> &marker)
{
    tf::StampedTransform transform;

    listener_.waitForTransform("map", "base_link",
                               ros::Time(0), ros::Duration(30.0));
    try
    {
        listener_.lookupTransform("map", "base_link",ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());
    //marker = transform;
    marker = ret;
}

void RobotDriver::QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler)
{
    btScalar W = TQuat.getW();
    btScalar X = TQuat.getX();
    btScalar Y = TQuat.getY();
    btScalar Z = TQuat.getZ();
    float WSquared = W * W;
    float XSquared = X * X;
    float YSquared = Y * Y;
    float ZSquared = Z * Z;
    TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
    TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
    TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
//   TEuler *= RADTODEG;
}

bool RobotDriver::driveInMap(tf::Stamped<tf::Pose> targetPoseMap,bool exitWhenStuck)
{

    tf::Stamped<tf::Pose> targetPoseBase;

    tf::Stamped<tf::Pose> startPose;
    getRobotPose(startPose);

    btVector3 diff = targetPoseMap.getOrigin() - startPose.getOrigin();
    //we dont move in z
    diff = btVector3(diff.x(), diff.y(), 0);
    float dist = diff.length();
    ROS_INFO("DISTANCE TO TRAVEL %f", dist);

    if (dist < 0.01)
        return true;

    ROS_INFO("diff unnorm %f %f %f ", diff.x(),diff.y(),diff.z());
    diff = (diff * ( 1.0 / dist)) ;
    ROS_INFO("diff norm %f %f %f ", diff.x(),diff.y(),diff.z());
    btVector3 rel = (dist + 0.05) * diff;
    ROS_INFO("new target in base %f %f", rel.x(), rel.y());

    ROS_INFO("TPM before adjust %f %f ", targetPoseMap.getOrigin().x(), targetPoseMap.getOrigin().y());
    targetPoseMap.setOrigin(startPose.getOrigin() + (dist + 0.02) * diff);
    ROS_INFO("TPM after adjust %f %f", targetPoseMap.getOrigin().x(), targetPoseMap.getOrigin().y());

    bool done = false;
    bool doneBad = false;
    int cnt = 0;
    int cntbad = 0;
    float lastDistMoved = 0;

    ROS_INFO("TARGET POSE IN MAP %f %f", targetPoseMap.getOrigin().x(), targetPoseMap.getOrigin().y());

    while (ros::ok() && !done)
    {
        cnt++;
        listener_.waitForTransform("base_link", "map",
                                   ros::Time(0), ros::Duration(30.0));

        targetPoseMap.stamp_ = ros::Time();
        targetPoseBase.stamp_ = ros::Time();

        listener_.transformPose("base_link", targetPoseMap, targetPoseBase);

        btVector3 euler;
        QuaternionToEuler(targetPoseBase.getRotation(),euler);


        //get the difference between the two poses
        //geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
        //tf_.transformPose(fixed_frame_, pose, fixed_pose);

        //wait for the listener to get the first message
        listener_.waitForTransform("base_footprint", "odom_combined",
                                   ros::Time(0), ros::Duration(10.0));

        //we will record transforms here
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;

        //record the starting transform from the odometry to the base frame
        listener_.lookupTransform("base_footprint", "odom_combined",
                                  ros::Time(0), start_transform);

        //we will be sending commands of type "twist"
        geometry_msgs::Twist base_cmd;

        //calulate speeds for reaching goal in 1 second and then scale them down to reasonable speeds
        float theoretic_x = targetPoseBase.getOrigin().x();
        float theoretic_y = targetPoseBase.getOrigin().y();
        float theoretic_w = euler.z();



        float scale_x = 1;
        float scale_y = 1;
        float scale_w = 1;
        if (fabs(theoretic_x) > 0.05)
            scale_x = 0.05 / fabs(theoretic_x);
        if (fabs(theoretic_y) > 0.05)
            scale_y = 0.05 / fabs(theoretic_y);
        if (fabs(theoretic_w) > 0.25)
            scale_w = 0.25 / fabs(theoretic_w);

        float scale = scale_x;
        if (scale_y < scale)
            scale = scale_y;
        if (scale_w < scale)
            scale = scale_w;

        //ROS_INFO("scales %f %f %f", scale_x, scale_y, scale_w);

        scale = fabs(scale);
        //if (scale > 1)
        //  scale = 1;

        base_cmd.linear.x = theoretic_x * scale;
        base_cmd.linear.y = theoretic_y * scale;
        base_cmd.angular.z = theoretic_w * scale;

        if ((scale == scale_w) && (fabs(theoretic_w) < 0.05))
            base_cmd.angular.z = theoretic_w * 0.05 / fabs(theoretic_w);

        //ROS_INFO("COMMAND %f %f %f", base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);

        ros::Rate rate(10.0);

        // if ((base_cmd.angular.z == 0) && (base_cmd.linear.y == 0) && (base_cmd.linear.x == 0))
        float distToTarg = sqrt(targetPoseBase.getOrigin().x() * targetPoseBase.getOrigin().x() + targetPoseBase.getOrigin().y() * targetPoseBase.getOrigin().y());
        //ROS_INFO("dist %f target pose in base coords: %f %f %f %f ANGLE %f ", distToTarg, targetPoseBase.getOrigin().x(), targetPoseBase.getOrigin().y(), targetPoseBase.getRotation().z(), targetPoseBase.getRotation().w(),euler.z());

        //if ((fabs(targetPoseBase.getOrigin().x()) < 0.02) && (fabs(targetPoseBase.getOrigin().y()) < 0.05) &&  (fabs(euler.z()) < 0.01))
        if (distToTarg < 0.02)
            done = true;
        //  while (!done && nh_.ok())

        //if (distToTarg > 0.3f) {

        tf::Stamped<tf::Pose> actPose;
        getRobotPose(actPose);
        float trav = 3 * (actPose.getOrigin() - startPose.getOrigin()).length();
        float used = (distToTarg < trav) ? distToTarg : trav;
        //float used = distToTarg;
        //! limit speed depening on already travelled distance (start slowly) and distance to goal (final approach slow)
        float approach_dist = 0.125; // 0.25;
        //if (used > 0.25f) {
        if (used > approach_dist)
        {
            //float used = (distToTarg < lastDistMoved) ? distToTarg : lastDistMoved;
            //float dscale = (distToTarg + .7) * (distToTarg + .7);
            float dscale = (used + (1-approach_dist)) * (used + (1-approach_dist)) * (used + (1-approach_dist));
            if (dscale > 5.0)
                dscale = 5.0;
            //ROS_INFO("USED %f TRAV %f TOTARG %f SCALE %f", used, trav, distToTarg, dscale);
            base_cmd.linear.x *= dscale;
            base_cmd.linear.y *= dscale;
            base_cmd.angular.z *= dscale;
        }
        else
        {
            //ROS_INFO("USED %f TRAV %f TOTARG %f", used, trav, distToTarg);
        }


        //send the drive command after safety check
        //if ((fabs(base_cmd.linear.x) <= 0.051) && (fabs(base_cmd.linear.y) <= 0.051) && (fabs(base_cmd.angular.z) <= 0.26))
        //if ((fabs(base_cmd.linear.x) <= 0.153) && (fabs(base_cmd.linear.y) <= 0.153) && (fabs(base_cmd.angular.z) <= 0.78))
        if ((fabs(base_cmd.linear.x) <= 0.255) && (fabs(base_cmd.linear.y) <= 0.255) && (fabs(base_cmd.angular.z) <= 1.3))
            cmd_vel_pub_.publish(base_cmd);
        else
            ROS_ERROR("COMMAND EXCEEDS MAXIMUM LIMITS (COMMAND %f %f %f)", base_cmd.linear.x, base_cmd.linear.y, base_cmd.angular.z);
        rate.sleep();
        //get the current transform
        try
        {
            listener_.lookupTransform("base_footprint", "odom_combined",
                                      ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            break;
        }
        //see how far we've traveled
        tf::Transform relative_transform = start_transform.inverse() * current_transform;

        double dist_moved = relative_transform.getOrigin().length();
        lastDistMoved = dist_moved;
        //ROS_INFO("dist_moved %f", dist_moved);
        // if we didnt move much and tried sometimes already, get out, we're stuck
        if ((dist_moved < 0.005) && (cnt < 15))
            cntbad++;
        else
            cntbad = 0;

        if (cntbad > 10)
        {
            ROS_ERROR("RobotDriver: could not reach goal, stuck");

            if (exitWhenStuck)
            {
                done = true;
                doneBad = true;
            }
        }
//      if(dist_moved > distance) done = true;
    }


    tf::Stamped<tf::Pose> actPose;
    getRobotPose(actPose);
    float travelled = (actPose.getOrigin() - startPose.getOrigin()).length();
    //ROS_ERROR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    btVector3 euler;
    ROS_INFO("TRAVELLED %f", travelled);
    QuaternionToEuler(targetPoseMap.getRotation(),euler);
    ROS_INFO("Goal in Map: %f %f %f", targetPoseMap.getOrigin().x(), targetPoseMap.getOrigin().y(), euler.z());
    QuaternionToEuler(actPose.getRotation(),euler);
    ROS_INFO("Curr in Map: %f %f %f", actPose.getOrigin().x(), actPose.getOrigin().y(), euler.z());
    //if (done) return true;
    return true;
}


bool RobotDriver::driveInMap(const float targetPose[], bool exitWhenStuck)
{
    tf::Stamped<tf::Pose> targetPoseMap;
    targetPoseMap.frame_id_ = "map";
    targetPoseMap.stamp_ = ros::Time();
    targetPoseMap.setOrigin(btVector3( targetPose[0], targetPose[1], 0));
    targetPoseMap.setRotation(btQuaternion(0,0, targetPose[2],  targetPose[3]));

    driveInMap(targetPoseMap,exitWhenStuck);
}

bool RobotDriver::driveInOdom(const float targetPose[], bool exitWhenStuck)
{

    listener_.waitForTransform("base_link", "map",
                               ros::Time(0), ros::Duration(10.0));

    tf::Stamped<tf::Pose> targetPoseMap;
    targetPoseMap.frame_id_ = "base_link";
    targetPoseMap.stamp_ = ros::Time();
    targetPoseMap.setOrigin(btVector3( targetPose[0], targetPose[1], 0));
    targetPoseMap.setRotation(btQuaternion(0,0, targetPose[2],  targetPose[3]));

    tf::Stamped<tf::Pose> targetPoseBase;

    targetPoseBase.stamp_ = ros::Time();

    listener_.transformPose("map", targetPoseMap, targetPoseBase);

    float pose[4];
    pose[0] = targetPoseBase.getOrigin().x();
    pose[1] = targetPoseBase.getOrigin().y();
    pose[2] = targetPoseBase.getRotation().z();
    pose[3] = targetPoseBase.getRotation().w();

    return driveInMap(pose,exitWhenStuck);
}



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void RobotDriver::moveBase(const float pose[], bool useNavigation)
{
    float x = pose[0];
    float y = pose[1];
    float oz = pose[2];
    float ow = pose[3];

    //RobotArm *arm = RobotArm::getInstance();
    // Start the trajectory

    //if (!arm->isTucked())
    //{
    //arm->startTrajectory(arm->lookAtMarker(Poses::untuckPoseB, Poses::untuckPoseB));
    ///  //arm->startTrajectory(arm->lookAtMarker(Poses::untuckPoseA, Poses::tuckPose));
    //}

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    //while(ros::ok() && !ac.waitForServer(ros::Duration(5.0))){
    //  ROS_INFO("Waiting for the move_base action server to come up");
    //}

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = oz;
    goal.target_pose.pose.orientation.w = ow;

    bool use_move_base = useNavigation;

    if (use_move_base)
    {
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        //bool finished_before_timeout =
        ac.waitForResult(ros::Duration(30.0));

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved");
        else
            ROS_INFO("The base failed to movefor some reason");
    }

    RobotDriver *driver = RobotDriver::getInstance();

    driver->driveInMap(pose);
}

void RobotDriver::moveBaseP(float x, float y, float oz, float ow, bool useNavigation)
{
    float p[4];
    p[0] = x;
    p[1] = y;
    p[2] = oz;
    p[3] = ow;
    moveBase(p,useNavigation);
}

void RobotDriver::driveToMatch(std::vector<tf::Stamped<tf::Pose> > targetPose, std::vector<std::string> frame_ids)
{
    ros::Rate rate(5);
    while (ros::ok())
    {

        rate.sleep();
        ros::spinOnce();

        RobotArm *right = RobotArm::getInstance(0);
        RobotArm *left = RobotArm::getInstance(1);
        tf::Stamped<tf::Pose> rightToolPose;
        right->getToolPose(rightToolPose, "base_link");
        tf::Stamped<tf::Pose> leftToolPose;
        left->getToolPose(leftToolPose, "base_link");


        tf::Stamped<tf::Pose> goalPoseRight;
        goalPoseRight.stamp_ = ros::Time::now();
        goalPoseRight.frame_id_= "map";
        goalPoseRight.setOrigin(btVector3(-1.657, 1.727, 0.982));
        goalPoseRight.setRotation(btQuaternion(-0.618, -0.312, 0.674, 0.256));
        goalPoseRight = RobotArm::getPoseIn("base_link",goalPoseRight);

        tf::Stamped<tf::Pose> goalPoseLeft;
        goalPoseLeft.stamp_ = ros::Time::now();
        goalPoseLeft.frame_id_= "map";
        goalPoseLeft.setOrigin(btVector3(-1.679, 1.987, 0.971));
        goalPoseLeft.setRotation(btQuaternion(-0.285, 0.654, -0.282, 0.641));
        goalPoseLeft = RobotArm::getPoseIn("base_link",goalPoseLeft);


        /*- Translation: [-1.657, 1.727, 0.982]
        - Rotation: in Quaternion [-0.618, -0.312, 0.674, 0.256]
            in RPY [-1.516, 0.739, 1.713]
        ruehr@satie:~/sandbox/tumros-internal/highlevel/ias_drawer_executive$ rosrun tf tf_echo map r_gripper_tool_frame
        At time 1286556189.961
        - Translation: [-1.679, 1.987, 0.971]
        - Rotation: in Quaternion [-0.285, 0.654, -0.282, 0.641]
            in RPY [-1.597, 0.746, -1.592]
        At time 1286556191.008*/


        //tf::Stamped<tf::Pose> rightTarget = targetPose[1];
        //Stamped<tf::Pose> leftTarget = targetPose[1];

        //::Transform relative_transform = start_transform/inverse() * current_transform;
        tf::Pose relative_transform_left;// = goalPoseLeft.inverse() * rightToolPose;
        tf::Pose relative_transform_right;// = goalPoseRight.inverse() * leftToolPose;

        relative_transform_right.setOrigin(rightToolPose.getOrigin() - goalPoseLeft.getOrigin());
        relative_transform_left.setOrigin(leftToolPose.getOrigin() - goalPoseRight.getOrigin());

        btVector3 average = relative_transform_left.getOrigin() + relative_transform_right.getOrigin();

        ROS_INFO("RELATIVE LEFT %f %f %f  %f %f %f %f", relative_transform_left.getOrigin().x(),relative_transform_left.getOrigin().y(),relative_transform_left.getOrigin().z(),
                 relative_transform_left.getRotation().x(), relative_transform_left.getRotation().y(), relative_transform_left.getRotation().z(), relative_transform_left.getRotation().w());
        ROS_INFO("RELATIVE RIGHT %f %f %f  %f %f %f %f", relative_transform_right.getOrigin().x(),relative_transform_right.getOrigin().y(),relative_transform_right.getOrigin().z(),
                 relative_transform_right.getRotation().x(), relative_transform_right.getRotation().y(), relative_transform_right.getRotation().z(), relative_transform_right.getRotation().w());

        ROS_INFO("AVERAGE %f %f", average.x(), average.y());
    }

}

RobotDriver *RobotDriver::instance = 0;



