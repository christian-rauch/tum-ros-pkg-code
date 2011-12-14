
#include <ias_drawer_executive/Geometry.h>

tf::TransformListener *Geometry::listener_=0;

void Geometry::init()
{
    if (!listener_)
        listener_ = new tf::TransformListener();
}

tf::Stamped<tf::Pose> Geometry::scaleStampedPose(const tf::Stamped<tf::Pose> &in, double scale)
{
    tf::Stamped<tf::Pose>  ret;
    ret.setRotation(btQuaternion::getIdentity().slerp(in.getRotation(), scale));
    ret.setOrigin(in.getOrigin() * scale);
    return ret;
}

tf::Stamped<tf::Pose> Geometry::scaleStampedTransform(const tf::Stamped<tf::Pose> &in, double scale)
{
    tf::Stamped<tf::Pose>  ret;
    ret.setRotation(btQuaternion::getIdentity().slerp(in.getRotation(), scale));
    ret.setOrigin(in.getOrigin() * scale);
    return ret;
}


tf::Stamped<tf::Pose> Geometry::getPoseIn(const char target_frame[], tf::Stamped<tf::Pose>src)
{

    if (src.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
    {
        ROS_ERROR("Frame not in TF: %s", src.frame_id_.c_str());
        tf::Stamped<tf::Pose> pose;
        return pose;
    }

    if (!listener_)
        listener_ = new tf::TransformListener();

    tf::Stamped<tf::Pose> transform;
    //this shouldnt be here TODO
    src.stamp_ = ros::Time(0);

    listener_->waitForTransform(src.frame_id_, target_frame,
                                ros::Time(0), ros::Duration(30.0));
    bool transformOk = false;
    while (!transformOk)
    {
        try
        {
            transformOk = true;
            listener_->transformPose(target_frame, src, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getPoseIn %s",ex.what());
            // dirty:
            src.stamp_ = ros::Time(0);
            transformOk = false;
        }
        ros::spinOnce();
    }
    return transform;
}

void Geometry::printPose(const tf::Stamped<tf::Pose> &toolTargetPose)
{
    ROS_INFO("%f %f %f %f %f %f %f %s\n",
             toolTargetPose.getOrigin().x(),  toolTargetPose.getOrigin().y(),  toolTargetPose.getOrigin().z(),toolTargetPose.getRotation().x(),
             toolTargetPose.getRotation().y(),toolTargetPose.getRotation().z(),toolTargetPose.getRotation().w(),toolTargetPose.frame_id_.c_str());
}



tf::Stamped<tf::Pose> Geometry::getPose(const char target_frame[],const char lookup_frame[])
{

    init();
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;

    listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(target_frame, lookup_frame,ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getPose: tf::TransformException ex.what()='%s'",ex.what());
            transformOk = false;
            listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
        }
        if (!transformOk)
            rate.sleep();
    }
    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}



tf::Stamped<tf::Pose> Geometry::getRelativeTransform(const char source_frameid[], const char target_frameid[])
{
    tf::StampedTransform transform;

    init();

    listener_->waitForTransform(source_frameid, target_frameid,
                                ros::Time(), ros::Duration(10.0));

    try
    {
        listener_->lookupTransform(std::string(source_frameid), std::string(target_frameid), ros::Time(), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("getTransformIn %s",ex.what());
    }

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

tf::Stamped<tf::Pose> Geometry::getTransform(const char baseframe[],  const char toolframe[])
{
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(baseframe, toolframe,ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getTransform %s",ex.what());
            transformOk = false;
        }
        rate.sleep();
    }
    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

tf::Stamped<tf::Pose> Geometry::rotateAroundBaseAxis(tf::Stamped<tf::Pose> toolPose, double r_x,double r_y,double r_z)
{
    tf::Stamped<tf::Pose> toolRotPlus;
    toolRotPlus.setOrigin(btVector3(0,0,0));
    toolRotPlus.setRotation(btQuaternion(r_x,r_y,r_z));
    btVector3 orig = toolPose.getOrigin();
    toolPose.setOrigin(btVector3(0,0,0));
    toolRotPlus *= toolPose;
    toolPose.setOrigin(orig);
    toolPose.setRotation(toolRotPlus.getRotation());
    return toolPose;
}

tf::Stamped<tf::Pose> Geometry::rotateAroundToolframeAxis(tf::Stamped<tf::Pose> toolPose, double r_x,double r_y,double r_z)
{
    tf::Stamped<tf::Pose> toolRotPlus;
    toolRotPlus.setOrigin(btVector3(0,0,0));
    toolRotPlus.setRotation(btQuaternion(r_x,r_y,r_z));
    btVector3 orig = toolPose.getOrigin();
    toolPose *= toolRotPlus;
    return toolPose;
}


tf::Stamped<tf::Pose> Geometry::rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, double r_x, double r_y, double r_z)
{
    btTransform curr = toolPose;
    btTransform pivo = pivot;

    curr = pivo.inverseTimes(curr);

    btQuaternion qa;
    qa.setEulerZYX(r_z,r_y,r_x);

    btTransform rot;
    rot.setOrigin(btVector3(0,0,0));
    rot.setRotation(qa);
    curr = rot * curr;
    curr = pivo * curr;

    tf::Stamped<tf::Pose> act;
    act.frame_id_ = toolPose.frame_id_;
    act.setOrigin(curr.getOrigin());
    act.setRotation(curr.getRotation());

    return act;
}


tf::Stamped<tf::Pose> Geometry::rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, btQuaternion qa)
{
    btTransform curr = toolPose;
    btTransform pivo = pivot;

    curr = pivo.inverseTimes(curr);

    btTransform rot;
    rot.setOrigin(btVector3(0,0,0));
    rot.setRotation(qa);
    curr = rot * curr;
    curr = pivo * curr;

    tf::Stamped<tf::Pose> act;
    act.frame_id_ = toolPose.frame_id_;
    act.setOrigin(curr.getOrigin());
    act.setRotation(curr.getRotation());

    return act;
}


tf::Stamped<tf::Pose> Geometry::approach(tf::Stamped<tf::Pose> toolPose, double dist)
{
    tf::Stamped<tf::Pose> appr;
    appr.stamp_ = ros::Time();
    appr.setOrigin(btVector3(- dist,0,0));
    appr.setRotation(btQuaternion(0,0,0,1));
    tf::Stamped<tf::Pose> ret;
    ret = toolPose;
    ret *= appr;
    return ret;
}


tf::Stamped<tf::Pose> Geometry::make_pose(double x, double y, double z, double ox, double oy, double oz, double ow, const char target_frame[])
{
    tf::Stamped<tf::Pose> toolTargetPose;

    toolTargetPose.frame_id_ = target_frame;
    toolTargetPose.stamp_ = ros::Time(0);
    toolTargetPose.setOrigin(btVector3( x,y,z));
    toolTargetPose.setRotation(btQuaternion(ox,oy,oz,ow));

    return toolTargetPose;
}

tf::Stamped<tf::Pose> Geometry::make_pose(const btTransform &trans, const char target_frame[])
{
    tf::Stamped<tf::Pose> toolTargetPose;
    toolTargetPose.frame_id_ = target_frame;
    toolTargetPose.stamp_ = ros::Time(0);
    toolTargetPose.setOrigin(trans.getOrigin());
    toolTargetPose.setRotation(trans.getRotation());

    return toolTargetPose;
}

tf::Stamped<tf::Pose> Geometry::getRel(const tf::Stamped<tf::Pose> &root_frame,  const tf::Stamped<tf::Pose> &relative_pose)
{
    btTransform rootbt;
    rootbt.setOrigin(root_frame.getOrigin());
    rootbt.setRotation(root_frame.getRotation());
    btTransform relativebt;
    relativebt.setOrigin(relative_pose.getOrigin());
    relativebt.setRotation(relative_pose.getRotation());

    btTransform retbt = rootbt * relativebt;

    tf::Stamped<tf::Pose> ret = root_frame;
    ret.setOrigin(retbt.getOrigin());
    ret.setRotation(retbt.getRotation());

    return ret;
}


tf::Stamped<tf::Pose> Geometry::getRelInBase(const tf::Stamped<tf::Pose> &root_frame,  const btVector3 &dist)
{
    tf::Stamped<tf::Pose> ret = root_frame;
    ret = getPoseIn("/base_link",ret);
    ret.getOrigin() += dist;
    ret = getPoseIn(root_frame.frame_id_.c_str(),ret);
    return ret;
}


void Geometry::printPose(const char title[], tf::Stamped<tf::Pose> pose)
{
    ROS_INFO("[ rosrun tf static_transform_publisher %f %f %f %f %f %f %f %s %s 100 ]", pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()
             , pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w(), pose.frame_id_.c_str(), title);
}
