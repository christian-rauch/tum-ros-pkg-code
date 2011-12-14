


#ifndef __OBJECTLOCALIZER_H__
#define __OBJECTLOCALIZER_H__

#include <map>
#include <string>
#include <tf/tf.h>

#include <ias_drawer_executive/Keywords.h>

class ObjectLocalizer;

typedef std::map<std::string, ObjectLocalizer*> localizer_map;

class ObjectLocalizer
{
public:

    ObjectLocalizer(std::string object_type);

    static bool localize(std::string object_class, tf::Stamped<tf::Pose> *poses, int numHits = 1, Keywords keys = Keywords());

    virtual bool localize_imp(std::string object_class, tf::Stamped<tf::Pose> *poses, int numHits = 1, Keywords keys = Keywords());

private:

    std::string object_type_;

    static localizer_map localizers;
};


#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class PotLocalizer : public ObjectLocalizer
{
public :

    PotLocalizer();

    virtual bool localize_imp(std::string object_class, tf::Stamped<tf::Pose> *poses, int numHits = 1, Keywords keys = Keywords());

    void getPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox, btVector3 min, btVector3 max);

    bool getCirclesFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pot_cyl_cloud,
                         double radius_goal,
                         double radius_tolerance,
                         std::vector<tf::Vector3> &center,
                         std::vector<double> &radius,
                         std::vector<int> &numinliers,
                         size_t  iterations = 5);

    bool getHandleRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &potPose,double pot_handle_radius_min, double pot_handle_radius_max, btVector3 min, btVector3 max);

    // min.z should be the table height, while min->max should span a box where all points necessary for detection lie in
    bool getLidPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &lid,double radius, tf::Vector3 min, tf::Vector3 max);

    // min.z should be the table height, while min->max should span a box where all points necessary for detection lie in
    bool getPotPoseViaLid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &potPose, tf::Vector3 min, tf::Vector3 max);

};




#endif
