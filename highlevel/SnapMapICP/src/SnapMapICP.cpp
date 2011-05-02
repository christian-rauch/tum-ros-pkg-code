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

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
//for point_cloud::fromROSMsg
#include <pcl/ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

//these should be parameters
// defines how good the match has to be to create a candidate for publishing a pose
float ICP_FITNESS_THRESHOLD = 100.1;// =  0.025;
//defines how much distance deviation from amcl to icp pose is needed to make us publish a pose
float DIST_THRESHOLD = 0.05;
// same for angle
float ANGLE_THRESHOLD = 0.01;


ros::NodeHandle *nh = 0;
ros::Publisher pub_output_;
ros::Publisher pub_output_scan;
ros::Publisher pub_output_scan_transformed;

ros::Publisher pub_pose;


laser_geometry::LaserProjection *projector_ = 0;
tf::TransformListener *listener_ = 0;
sensor_msgs::PointCloud2 cloud2;
sensor_msgs::PointCloud2 cloud2transformed;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

boost::shared_ptr< sensor_msgs::PointCloud2> output_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
boost::shared_ptr< sensor_msgs::PointCloud2> scan_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());

bool we_have_a_map = false;
bool we_have_a_scan = false;
bool we_have_a_scan_transformed = false;

int lastScan = 0;
int actScan = 0;

/*inline void
  pcl::transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}*/

inline void
matrixAsTransfrom (const Eigen::Matrix4f &out_mat,  tf::Transform& bt)
{
    double mv[12];

    mv[0] = out_mat (0, 0) ;
    mv[4] = out_mat (0, 1);
    mv[8] = out_mat (0, 2);
    mv[1] = out_mat (1, 0) ;
    mv[5] = out_mat (1, 1);
    mv[9] = out_mat (1, 2);
    mv[2] = out_mat (2, 0) ;
    mv[6] = out_mat (2, 1);
    mv[10] = out_mat (2, 2);

    btMatrix3x3 basis;
    basis.setFromOpenGLSubMatrix(mv);
    btVector3 origin(out_mat (0, 3),out_mat (1, 3),out_mat (2, 3));

    ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

    bt = tf::Transform(basis,origin);
}


void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    ROS_DEBUG("I heard frame_id: [%s]", msg.header.frame_id.c_str());

    float resolution = msg.info.resolution;
    float width = msg.info.width;
    float height = msg.info.height;

    float posx = msg.info.origin.position.x;
    float posy = msg.info.origin.position.y;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_xyz = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());

    //cloud_xyz->width    = 100; // 100
    cloud_xyz->height   = 1;
    cloud_xyz->is_dense = false;
    cloud_xyz->header.stamp = ros::Time(0);
    cloud_xyz->header.frame_id = "/map";

    pcl::PointXYZ point_xyz;

    //for (unsigned int i = 0; i < cloud_xyz->width ; i++)
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            //@TODO
            if (msg.data[x + y * width] == 100)
            {
                point_xyz.x = (.5f + x) * resolution + posx;
                point_xyz.y = (.5f + y) * resolution + posy;
                point_xyz.z = 0;
                cloud_xyz->points.push_back(point_xyz);
            }
        }
    cloud_xyz->width = cloud_xyz->points.size();

    pcl::toROSMsg (*cloud_xyz, *output_cloud);
    ROS_DEBUG("Publishing PointXYZ cloud with %ld points in frame %s", cloud_xyz->points.size(),output_cloud->header.frame_id.c_str());

    we_have_a_map = true;
}


int lastTimeSent = -1000;

int count_sc_ = 0;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

    //projector_.transformLaserScanToPointCloud("base_link",*scan_in,cloud,listener_);
    count_sc_++;
    if (count_sc_ > 10)
    {
        count_sc_ = 0;


        sensor_msgs::PointCloud cloud;
        sensor_msgs::PointCloud cloudInMap;

        projector_->projectLaser(*scan_in,cloud);

        we_have_a_scan = false;
        bool gotTransform = false;
        while (!gotTransform && (ros::ok()))
        {
            try
            {
                gotTransform = true;
                listener_->transformPointCloud ("/map",cloud,cloudInMap);
            }
            catch (...)
            {
                gotTransform = false;
            }
        }

        gotTransform = false;
        tf::StampedTransform oldPose;
        while (!gotTransform && (ros::ok()))
        {
            try
            {
                gotTransform = true;
                listener_->lookupTransform("/map", "/base_footprint",
                                           ros::Time(0), oldPose);
            }
            catch (tf::TransformException ex)
            {
                gotTransform = false;
            }
        }
        if (we_have_a_map && gotTransform)
        {
            sensor_msgs::convertPointCloudToPointCloud2(cloudInMap,cloud2);
            we_have_a_scan = true;

            actScan++;

            //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
            reg.setTransformationEpsilon (1e-6);
            // Set the maximum distance between two correspondences (src<->tgt) to 10cm
            // Note: adjust this based on the size of your datasets
            reg.setMaxCorrespondenceDistance(0.5);
            reg.setMaximumIterations (100);
            // Set the point representation

            PointCloudT myMapCloud;
            PointCloudT myScanCloud;

            pcl::fromROSMsg(*output_cloud,myMapCloud);
            pcl::fromROSMsg(cloud2,myScanCloud);

            reg.setInputCloud(boost::make_shared< const PointCloudT > (myScanCloud));
            reg.setInputTarget(boost::make_shared< const PointCloudT > (myMapCloud));

            PointCloudT unused;
            int i = 0;
            reg.align (unused); //-- running the registration in a loop to visualize intermediate results

            Eigen::Matrix4f transf = reg.getFinalTransformation();
            tf::Transform t;
            matrixAsTransfrom(transf,t);

            we_have_a_scan_transformed = false;
            PointCloudT transformedCloud;
            pcl::transformPointCloud (myScanCloud, transformedCloud, reg.getFinalTransformation());

            pcl::toROSMsg (transformedCloud, cloud2transformed);
            we_have_a_scan_transformed = true;


            double dist = sqrt((t.getOrigin().x() * t.getOrigin().x()) + (t.getOrigin().y() * t.getOrigin().y()));
            double angleDist = t.getRotation().getAngle();
            t =  t * oldPose ;

            //ROS_INFO("dist %f angleDist %f",dist, angleDist);


            if ((actScan - lastTimeSent > 10) && ((dist > DIST_THRESHOLD) || (angleDist > ANGLE_THRESHOLD)))
                if ( reg.getFitnessScore()  <= ICP_FITNESS_THRESHOLD )
                {
                    lastTimeSent = actScan;
                    geometry_msgs::PoseWithCovarianceStamped pose;
                    pose.header.frame_id = "map";
                    pose.pose.pose.position.x = t.getOrigin().x();
                    pose.pose.pose.position.y = t.getOrigin().y();

                    btQuaternion quat = t.getRotation();
                    //quat.setRPY(0.0, 0.0, theta);
                    tf::quaternionTFToMsg(quat,pose.pose.pose.orientation);
                    float factorPos = 0.03;
                    float factorRot = 0.1;
                    pose.pose.covariance[6*0+0] = (0.5 * 0.5) * factorPos;
                    pose.pose.covariance[6*1+1] = (0.5 * 0.5) * factorPos;
                    pose.pose.covariance[6*3+3] = (M_PI/12.0 * M_PI/12.0) * factorRot;
                    ROS_INFO("i %i converged %i SCORE: %f", i,  reg.hasConverged (),  reg.getFitnessScore()  );
                    ROS_INFO("PUBLISHING A NEW INITIAL POSE dist %f angleDist %f Setting pose: %.3f %.3f  [frame=%s]",dist, angleDist , pose.pose.pose.position.x  , pose.pose.pose.position.y , pose.header.frame_id.c_str());
                    pub_pose.publish(pose);
                }
            //ROS_DEBUG("map width %i height %i size %i, %s", myMapCloud.width, myMapCloud.height, (int)myMapCloud.points.size(), myMapCloud.header.frame_id.c_str());
            //ROS_DEBUG("scan width %i height %i size %i, %s", myScanCloud.width, myScanCloud.height, (int)myScanCloud.points.size(), myScanCloud.header.frame_id.c_str());
        }
    }
    // Do something with cloud.
}

int main(int argc, char** argv)
{

// Init the ROS node
    ros::init(argc, argv, "snapmapicp");
    ros::NodeHandle nh_;
    nh = &nh_;

    projector_ = new laser_geometry::LaserProjection();
    tf::TransformListener listener;
    listener_ = &listener;

    pub_output_ = nh->advertise<sensor_msgs::PointCloud2> ("map_points", 1);
    pub_output_scan = nh->advertise<sensor_msgs::PointCloud2> ("scan_points", 1);
    pub_output_scan_transformed = nh->advertise<sensor_msgs::PointCloud2> ("scan_points_transformed", 1);
    pub_pose = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    ros::Subscriber subMap = nh_.subscribe("map", 10, mapCallback);
    ros::Subscriber subScan = nh_.subscribe("base_scan", 10, scanCallback);

    ros::Rate loop_rate(10);

    listener_->waitForTransform("/base_link", "/map",
                                ros::Time(0), ros::Duration(30.0));

    listener_->waitForTransform("/base_laser_link", "/map",
                                ros::Time(0), ros::Duration(30.0));

    while (ros::ok())
    {
        if (actScan > lastScan)
        {
            lastScan = actScan;
            // publish map as a pointcloud2
            //if (we_have_a_map)
            //   pub_output_.publish(Ŕ);
            // publish scan as seen as a pointcloud2
            //if (we_have_a_scan)
            //   pub_output_scan.publish(cloud2);
            // publish icp transformed scan
            if (we_have_a_scan_transformed)
                pub_output_scan_transformed.publish(cloud2transformed);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

}


/*
Full name: /Motors/EtherCAT Device (r_wrist_l_motor)
Component: EtherCAT Device (r_wrist_l_motor)
Hardware ID: 68-05005-01769
Level: Warning
Message: Could not collect WG0X diagnostics; Could not collect diagnostics

Configuration: good
Name: r_wrist_l_motor
Position: 31
Product code: WG05 (6805005) Firmware Revision 1.21, PCB Revision F.02
Robot: PR2
Motor: Maxon 310009
Serial Number: 68-05005-01769
Nominal Current Scale: 0.000500
Nominal Voltage Scale: 0.006250
HW Max Current: 3.250000
SW Max Current: 1.720000
Speed Constant: 178.000000
Resistance: 2.520000
Motor Torque Constant: 0.053800
Pulses Per Revolution: 1200
Encoder Reduction: -1.000000
Status Checksum Error Count: 0
Safety Disable Status: ENABLED (00)
Safety Disable Status Hold: ENABLED (00)
Safety Disable Count: 6
Undervoltage Count: 5
Over Current Count: 0
Board Over Temp Count: 0
Bridge Over Temp Count: 0
Operate Disable Count: 1
Watchdog Disable Count: 1
MBX Command IRQ Count: 181
PDI Timeout Error Count: 0
PDI Checksum Error Count: 0
Supply Current: 0.051398
Configured Offset A: 0.022000
Configured Offset B: -0.006000
Mailbox Write Errors: 0
Mailbox Read Errors: 0
Mailbox Retries: 0
Mailbox Retry Errors: 0
Calibration Offset: -198.797365
Calibration Status: Calibrated by controller
Watchdog Limit: 100ms
Mode: ENABLE, CURRENT
Digital out: 254
Programmed pwm value: -291
Programmed current: -0.197000
Measured current: -0.194500
Timestamp: 3546451304
Encoder count: -38034
Encoder index pos: -38034
Num encoder_errors: 0
Encoder status: 15
Calibration reading: 15
Last calibration rising edge: -43040
Last calibration falling edge: -18976
Board temperature: 44.250000
Max board temperature: 44.375000
Bridge temperature: 50.687500
Max bridge temperature: 50.812500
Supply voltage: 70.787498
Motor voltage: -1.125000
Current Loop Kp: 16
Current Loop Ki: 4
Motor Voltage Error %: 10.172653
Max Motor Voltage Error %: 34.376442
Abs Filtered Voltage Error %: 8.273659
Max Abs Filtered Voltage Error %: 15.651860
Current Error: 0.000720
Max Current Error: 0.017975
Abs Filtered Current Error: 0.000570
Max Abs Filtered Current Error: 0.003362
Motor Resistance Estimate: 4.495118
# Published traces: 5
Packet count: 48769
Drops: 0
Consecutive Drops: 0
Max Consecutive Drops: 0
Reset detected: No
Valid: No
EPU Errors: 0
PDI Errors: 0
Status Port 0: Has Link, Open, Has Comm
RX Error Port 0: 0
Forwarded RX Error Port 0: 0
Invalid Frame Port 0: 0
Lost Link Port 0: 0
Status Port 1: Has Link, Open, Has Comm
RX Error Port 1: 0
Forwarded RX Error Port 1: 0
Invalid Frame Port 1: 0
Lost Link Port 1: 0
*/
