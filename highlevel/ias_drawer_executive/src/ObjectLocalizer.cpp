#include <ros/ros.h>
#include <ias_drawer_executive/ObjectLocalizer.h>


localizer_map ObjectLocalizer::localizers;

ObjectLocalizer::ObjectLocalizer(std::string object_type)
{
    object_type_ = object_type;
    //ROS_INFO("construced %s", object_type_.c_str());
    //ROS_INFO("localizers.size %zu", localizers.size());
    localizers[object_type] = this;
    //ROS_INFO("localizers.size %zu", localizers.size());
    //ROS_INFO("localizers[%s] %i", object_type.c_str(), localizers[object_type]);
    //ROS_INFO("localizers[%s] %i", "blabla", localizers["blabla"]);
    ROS_INFO("Adding Object Localizer for type %s", object_type.c_str());
}

bool ObjectLocalizer::localize(std::string object_class,tf::Stamped<tf::Pose> *poses, int numHits, Keywords keys)
{
    ROS_INFO("Looking for a %s", object_class.c_str());
    localizer_map::const_iterator hit;

    hit = localizers.find(object_class);

    if (hit == localizers.end())
    {
        ROS_ERROR("No Localizer registered for object type\"%s\"", object_class.c_str());
        return false;
    }
    else
        return (*hit).second->localize_imp(object_class,poses,numHits,keys);
}

bool ObjectLocalizer::localize_imp(std::string object_class, tf::Stamped<tf::Pose> *poses, int numHits, Keywords keys)
{
    ROS_ERROR("Not Implemented");
    return false;
}


// ============================================

#include <ias_drawer_executive/Kinect.h>

PotLocalizer::PotLocalizer()  : ObjectLocalizer("Pot")
{
    ROS_INFO("PotLocalizer constructor");
}

bool PotLocalizer::localize_imp(std::string object_class, tf::Stamped<tf::Pose> *poses, int numHits, Keywords keys)
{
    ROS_INFO("PotLocalizer::localize_imp");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    Kinect::getInstance()->getCloud(cloud,"/map");

    PotLocalizer::getPotPoseViaLid(cloud,poses[0],keys.lookup("min"),keys.lookup("max"));

    return true;
}


#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ias_drawer_executive/Geometry.h>



void PotLocalizer::getPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox, btVector3 min, const btVector3 max)
{

    Eigen::Vector4f min_pt, max_pt;

    min_pt = Eigen::Vector4f(min.x(),min.y(),min.z(), 1);
    max_pt = Eigen::Vector4f(max.x(),max.y(),max.z(), 1);

    ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
    ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);

    ROS_INFO("cloud size : %zu", cloud->points.size());

    boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
    pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

    pcl::ExtractIndices<pcl::PointXYZRGB> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*inBox);

    //pubCloud(inBox);

    ROS_INFO("cloud size after box filtering: %zu", inBox->points.size());
}



bool PotLocalizer::getCirclesFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pot_cyl_cloud, double radius_goal, double radius_tolerance,
                                       std::vector<tf::Vector3> &center,
                                       std::vector<double> &radius,
                                       std::vector<int> &numinliers,
                                       size_t  iterations)
{


    ros::NodeHandle node_handle;

    //ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);

    center.clear();
    radius.clear();
    numinliers.clear();

    center.resize(iterations);
    radius.resize(iterations);
    numinliers.resize(iterations);

    pcl::PointCloud<pcl::PointXYZRGB> act = *pot_cyl_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> flip;

    // in each iteration, the inliers of the last circle found are taken out and the sacsegmentation is run again
    for (size_t k = 0; k < iterations; k++)
    {

        //sensor_msgs::PointCloud2 out; //in map frame
        //pcl::toROSMsg(act,out);
        //out.header.frame_id = "/map";
        //cloud_pub.publish(out);

        //pubCloud(act.makeShared());

        ros::NodeHandle node_handle;
        ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (act.makeShared());
        seg.segment (*inliers, *coefficients);

        seg.setRadiusLimits(radius_goal - radius_tolerance, radius_goal + radius_tolerance);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return false;
        }
        else
        {
            double z = 0;
            for (size_t g=0; g < inliers->indices.size(); ++g)
            {
                z += act.points[inliers->indices[g]].z;
            }
            z /=  inliers->indices.size();
            ROS_INFO("MODEL :");
            //coefficients.get()
            ROS_INFO("%f %f %f %f SIZE : %zu", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3], inliers->indices.size());
            center[k] = tf::Vector3(coefficients->values[0], coefficients->values[1],z);
            radius[k] = coefficients->values[2];
            numinliers[k] = inliers->indices.size();

            tf::Stamped<tf::Pose> res;
            res.setOrigin(center[k]);
            res.setRotation(btQuaternion(0,0,0,1));
            res.frame_id_ = "/map";

            geometry_msgs::PoseStamped res_msg;
            tf::poseStampedTFToMsg(res,res_msg);

            pose_pub.publish(res_msg);

        }
        flip = act;
        pcl::ExtractIndices<pcl::PointXYZRGB> ei;
        ei.setInputCloud(flip.makeShared());
        ei.setIndices(inliers);
        ei.setNegative(true);
        ei.filter(act);
        //ros::Duration(1).sleep();
    }
    return true;
}


bool PotLocalizer::getHandleRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &potPose,double pot_handle_radius_min, double pot_handle_radius_max, btVector3 min, btVector3 max)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    getPointsInBox(cloud,cloud_filtered,min,max);

    // we will calculate the angle of each point that we think belongs to the handle via arctan and then average over that angles
    double atansum = 0;
    double atannum = 0;

    for (size_t k = 0; k < cloud_filtered->points.size(); ++k)
    {
        tf::Vector3 act(cloud_filtered->points[k].x,cloud_filtered->points[k].y,cloud_filtered->points[k].z);
        tf::Vector3 rel = potPose.getOrigin() - act;
        rel.setZ(0);
        double radius = rel.length();

        if ((radius > pot_handle_radius_min) && (radius < pot_handle_radius_max))
        {
            tf::Vector3 act(cloud_filtered->points[k].x- potPose.getOrigin().x(),cloud_filtered->points[k].y - potPose.getOrigin().y(),0);
            if (act.x() != 0.0)
            {
                double at = atan2(act.y(),act.x());

                //we want the rotation modulo 180deg
                if (at < 0)
                    at += M_PI;

                atansum += at;
                atannum += 1;
            }
        }
    }

    if (atannum < 20)
    {
        ROS_ERROR("did not get enough handle inliers");
        return false;
    }

    atansum /= atannum;

    potPose.setRotation(btQuaternion(tf::Vector3(0,0,1),atansum));
    return true;
}

bool PotLocalizer::getLidPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &lid, double radius_goal, tf::Vector3 min, tf::Vector3 max)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);

    getPointsInBox(cloud,inBox,min,max);

    //pubCloud(inBox);

    std::vector<tf::Vector3> center;
    std::vector<double> radius;
    std::vector<int> numinliers;

    // for now we assume we see only one circle fitting and this will be the one, todo: get multiple hyptheses and preserve them
    getCirclesFromCloud(inBox, radius_goal, .04,center,radius,numinliers,1);

    // sanity check, we get will usually see a lot more points
    if (numinliers[0] < 300)
        return false;

    lid.setOrigin(center[0]);
    lid.setRotation(btQuaternion(0,0,0,1));
    lid.frame_id_ = "/map";

    // TODO: dont do this here, for now this should be independent of the robot frame
    //lid = Geometry::getPoseIn("base_link", lid);
    //lid.setRotation(btQuaternion(0,0,1,0));
    //lid = Geometry::getPoseIn("map", lid);

    //announce("Lid", lid);

    return true;
}


bool PotLocalizer::getPotPoseViaLid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &potPose, tf::Vector3 min, tf::Vector3 max)
{
    double table_height = min.z();

    // some values for our magical pot, todo: move to parameters, xml or something
    double pot_cyl_min = 0.02;
    double pot_cyl_max = 0.05;
    double pot_handle_min = 0.06;
    double pot_handle_max = 0.09;
    double pot_radius = 0.20 / 2.0;
    double pot_handle_radius_max = 0.28 / 2.0;
    double pot_handle_radius_min = 0.25 / 2.0;

    //announce_box("pot search", roi_min, roi_max);
    //announce_box("pot search", roi_min, roi_max);

    ros::NodeHandle node_handle;

    ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/debug_cloud",0,true);
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detection", 0 , true);
    ros::Publisher pose_pubr = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionr", 0 , true);
    ros::Publisher pose_publ = node_handle.advertise<geometry_msgs::PoseStamped>( "/cop_detectionl", 0 , true);


    //! find the pot center
    tf::Vector3 center;

    tf::Stamped<tf::Pose> lidPose;
    //if (!getLidPose(cloud,lidPose, min + tf::Vector3(0,0,pot_handle_max), max))
    if (!getLidPose(cloud,lidPose, pot_radius, min + tf::Vector3(0,0,pot_cyl_min), tf::Vector3(max.x(), max.y(), min.z() + pot_cyl_max)))
    {
        ROS_ERROR("Could not find LID in Search Region!");
        return false;
    }
    center = lidPose.getOrigin();

    ROS_INFO("LID CENTER %f %f %f",center.x(),center.y(),center.z());


    //! find the handles on the pot side
    center.setZ(table_height);

    potPose.setOrigin(center);

    if (!getHandleRotation(cloud,potPose,pot_handle_radius_min, pot_handle_radius_max, center - tf::Vector3(pot_handle_radius_max,pot_handle_radius_max,-pot_handle_min), center + tf::Vector3(pot_handle_radius_max,pot_handle_radius_max,pot_handle_max)))
    {
        ROS_ERROR("Could not get Hanndle Orientation");
        return false;
    }

    Geometry::printPose("Pot Pose", potPose);

    return true;
}






