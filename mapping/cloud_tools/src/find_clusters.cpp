// #include <unistd.h>

#include <ctime>

#include <ros/node_handle.h>


#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>
#include <angles/angles.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_table_msgs/TableWithObjects.h>

// Comparison operator for a vector of vectors
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class FindClusters
{
  protected:
    ros::NodeHandle nh_;
    std::string input_cloud_topic_;
    ros::Subscriber cloud_sub_;
    tf::TransformListener tf_;
    mapping_msgs::PolygonalMap pmap_;

    //
    ias_table_msgs::TableWithObjects table;
    ros::Publisher table_pub_;
    ros::Publisher cloud_pub_;
    
    // ROS parameters:
    double object_cluster_tolerance_;
    int object_cluster_min_pts_;
    int k_;
    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;
    double clusters_growing_tolerance_;
    int clusters_min_pts_;
    double table_min_height_, table_max_height_, delta_z_, object_min_distance_from_table_;


    // other vars
    geometry_msgs::Point32 z_axis_;
 

  public:
    FindClusters ()
    {
      nh_.param ("sac_distance_threshold_", sac_distance_threshold_, 0.02); 
      nh_.param ("table_min_height_", table_min_height_, 0.5);   // 0.5 m
      nh_.param ("table_max_height_", table_max_height_, 1.5);   // 0.5 m
      nh_.param ("clusters_growing_tolerance", clusters_growing_tolerance_, 0.5);   // 0.5 m
      nh_.param ("clusters_min_pts", clusters_min_pts_, 10);                        // 10 points
      nh_.param ("table_delta_z", delta_z_, 0.03);                         // consider objects starting at 3cm from the table
      nh_.param ("object_min_distance_from_table", object_min_distance_from_table_, 0.10); // objects which have their support more 10cm from the table will not be considered

        nh_.param ("object_cluster_dist_tolerance", object_cluster_tolerance_, 0.05);   // 5cm between two objects
        nh_.param ("~object_cluster_min_pts", object_cluster_min_pts_, 30);              // minimum 30 points per object cluster
      nh_.param ("normal_eps_angle", eps_angle_, 15.0);                   // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);                        // convert to radians
 
      nh_.param ("region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = angles::from_degrees (region_angle_threshold_); // convert to radians
      
      k_ = 20;
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("shoulder_cloud"));       // 15 degrees
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &FindClusters::cloud_cb, this);
      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      table_pub_ = nh_.advertise<ias_table_msgs::TableWithObjects> ("table_with_objects", 1);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> ("debug_cloud_pcd", 1);
    }
   
    void
      estimatePointNormals (sensor_msgs::PointCloud &cloud)
    {
      ROS_INFO ("+ estimatePointNormals, %i", cloud.points.size ());
      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (cloud);
      std::vector<std::vector<int> > points_k_indices;
      ROS_INFO ("1 estimatePointNormals");
      // Allocate enough space for point indices
      points_k_indices.resize (cloud.points.size ());
      ROS_INFO ("2 estimatePointNormals %i", k_);
      for (int i = 0; i < (int)cloud.points.size (); i++)
        points_k_indices[i].resize (k_);
      // Get the nerest neighbors for all the point indices in the bounds
      ROS_INFO ("3 estimatePointNormals");
      std::vector<float> distances;
      for (int i = 0; i < (int)cloud.points.size (); i++)
        kdtree->nearestKSearch (i, k_, points_k_indices[i], distances);
    
      ROS_INFO ("4 estimatePointNormals");
      // Figure out the viewpoint value in the point cloud frame
      geometry_msgs::PointStamped viewpoint_laser, viewpoint_cloud;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0,0,0
      ROS_INFO ("5 estimatePointNormals");
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;
    
      try
      {
        tf_.transformPoint (cloud.header.frame_id, viewpoint_laser, viewpoint_cloud);
      }
      catch (tf::TransformException)
      {
        viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
      }
  
      ROS_INFO ("5 estimatePointNormals");
      //#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)cloud.points.size (); i++)
      {
        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        Eigen::Vector4d plane_parameters;
        double curvature;
        cloud_geometry::nearest::computePointNormal (cloud, points_k_indices[i], plane_parameters, curvature);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud.points[i], viewpoint_cloud);

        cloud.channels[0].values[i] = plane_parameters (0);
        cloud.channels[1].values[i] = plane_parameters (1);
        cloud.channels[2].values[i] = plane_parameters (2);
      }
      // Delete the kd-tree
      delete kdtree;
      ROS_INFO ("- estimatePointNormals");
    }
    

    std::vector <int> 
      pointsInZBounds (const sensor_msgs::PointCloudConstPtr& cloud_in_)
    {
      ROS_INFO ("+ pointsInZBounds");
      // find points within bounds..
      std::vector<int> indices_in_bounds (cloud_in_->points.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < cloud_in_->points.size (); i++)
      {
        if (cloud_in_->points[i].z >= table_min_height_ && cloud_in_->points[i].z <= table_max_height_)
        {
          indices_in_bounds[nr_p] = i;
          nr_p++;
        }
      }
      indices_in_bounds.resize (nr_p);
      ROS_INFO ("%d of %d points are within the table height bounds of [%.2lf,%.2lf]",
                 nr_p, (int)cloud_in_->points.size (), table_min_height_, table_max_height_);
      ROS_INFO ("- pointsInZBounds");
      return indices_in_bounds;
    }
    
    sensor_msgs::PointCloud
      downsampleCloud (const sensor_msgs::PointCloudConstPtr& cloud_in_, std::vector<int> indices_in_bounds)
    {
      ROS_INFO ("+ downsampleCloud");
      sensor_msgs::PointCloud cloud_down_;
      geometry_msgs::Point leaf_width_;
      leaf_width_.x = leaf_width_.y = leaf_width_.z = 0.05;
      
      std::vector<cloud_geometry::Leaf> leaves;
      try
      {
        cloud_geometry::downsamplePointCloud (*cloud_in_, indices_in_bounds, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (*cloud_in_);
      }
      ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_down_.points.size ());
      // Reserve space for 3 channels: nx, ny, nz
      cloud_down_.channels.resize (3);
      cloud_down_.channels[0].name = "nx";
      cloud_down_.channels[1].name = "ny";
      cloud_down_.channels[2].name = "nz";
      for (unsigned int d = 0; d < cloud_down_.channels.size (); d++)
        cloud_down_.channels[d].values.resize (cloud_down_.points.size ());
      ROS_INFO ("- downsampleCloud");
      return cloud_down_;
    }

    void
      findObjectClusters (sensor_msgs::PointCloud &points, const std::vector<double> &coeff, const geometry_msgs::Polygon &poly,
                          const geometry_msgs::Point32 &minP, const geometry_msgs::Point32 &maxP,
                          std::vector<int> &object_indices)
    {
      int nr_p = 0;
      geometry_msgs::Point32 pt;
      object_indices.resize (points.points.size ());
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        // Select all the points in the given bounds
        if ( points.points.at (i).x > minP.x &&
             points.points.at (i).x < maxP.x &&
             points.points.at (i).y > minP.y &&
             points.points.at (i).y < maxP.y &&
             points.points.at (i).z > (maxP.z + delta_z_)
           )
        {
          // Calculate the distance from the point to the plane
          double distance_to_plane = coeff.at (0) * points.points.at (i).x +
                                     coeff.at (1) * points.points.at (i).y +
                                     coeff.at (2) * points.points.at (i).z +
                                     coeff.at (3) * 1;
          // Calculate the projection of the point on the plane
          pt.x = points.points.at (i).x - distance_to_plane * coeff.at (0);
          pt.y = points.points.at (i).y - distance_to_plane * coeff.at (1);
          pt.z = points.points.at (i).z - distance_to_plane * coeff.at (2);

          if (cloud_geometry::areas::isPointIn2DPolygon (pt, poly))
          {
            object_indices[nr_p] = i;
            nr_p++;
          }
        }
      }
      object_indices.resize (nr_p);

      // Find the clusters
      nr_p = 0;
      std::vector<std::vector<int> > object_clusters;
      cloud_geometry::nearest::extractEuclideanClusters (points, object_indices, object_cluster_tolerance_, object_clusters, -1, -1, -1, -1, object_cluster_min_pts_);

      geometry_msgs::Point32 minPCluster, maxPCluster;
      table.point_clusters.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        std::vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (points, object_idx, minPCluster, maxPCluster);
        if (minPCluster.z > (maxP.z + object_min_distance_from_table_) )
            continue;

        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          object_indices[nr_p] = object_idx.at (j);
          nr_p++;
        }
        cloud_geometry::statistics::getMinMax (points, object_idx, table.objects[i].min_bound, table.objects[i].max_bound);
        cloud_geometry::nearest::computeCentroid (points, object_idx, table.objects[i].center);
      }
      object_indices.resize (nr_p);
    }
    
    bool detectPlanes (const sensor_msgs::PointCloudConstPtr& cloud_in_)
    {
      ROS_INFO ("+ detectPlanes");
/// ----------------------------------------------------------------------- Z BOUNDS
/// ----------------------------------------------------------------------- DOWNSAMPLE
      sensor_msgs::PointCloud cloud_down_ = downsampleCloud (cloud_in_, pointsInZBounds (cloud_in_));
/// ----------------------------------------------------------------------- NORMAL ESTIMATION
      // Create Kd-Tree
      estimatePointNormals (cloud_down_);
/// ----------------------------------------------------------------------- FIND HORIZONTAL CLUSTERS (normals based)
      // ---[ Select points whose normals are perpendicular to the Z-axis
      std::vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_, z_axis_, indices_z);
      ROS_INFO ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());
      
      sensor_msgs::PointCloud temp_cloud;
      temp_cloud.header.frame_id = "robot_fake_base";
      for (unsigned int i = 0; i < indices_z.size(); i++)
        temp_cloud.points.push_back (cloud_down_.points[indices_z[i]]) ;
      cloud_pub_.publish (temp_cloud);

      std::vector<std::vector<int> > clusters;
      // Split the Z-parallel points into clusters
      cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, indices_z, clusters_growing_tolerance_, clusters, 0, 1, 2, region_angle_threshold_, clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);

      std::vector<int> inliers;
      std::vector<double> coeff, z_coeff (3);
      z_coeff[0] = z_axis_.x; z_coeff[1] = z_axis_.y; z_coeff[2] = z_axis_.z;
      int c_good = -1;
      double eps_angle_deg = angles::to_degrees (eps_angle_);
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        // Find the best plane in this cluster
        fitSACPlane (&cloud_down_, &clusters[i], inliers, coeff);
      ROS_INFO ("after! fitSACPlane");
      if (inliers.size() == 0 || coeff.size() == 0)
        continue;

        double angle = angles::to_degrees (cloud_geometry::angles::getAngleBetweenPlanes (coeff, z_coeff));
        if ( fabs (angle) < eps_angle_deg || fabs (180.0 - angle) < eps_angle_deg )
        {
          c_good = i;
          break;
        }
      }

      if (c_good == -1)
      {
        ROS_WARN ("No table found");
        return (false);
      }
      ROS_INFO ("Number of clusters found: %d, largest cluster: %d.", (int)clusters.size (), (int)clusters[c_good].size ());
/// ----------------------------------------------------------------------- TABLE TRANSFORM... ??
      // Fill in the header
//       resp.table.header.frame_id = global_frame_;
//       resp.table.header.stamp = cloud_in_.header.stamp;
//
//      // Get the table bounds
      geometry_msgs::Point32 minP, maxP;
      cloud_geometry::statistics::getMinMax (cloud_down_, inliers, minP, maxP);
//      // Transform to the global frame
//      geometry_msgs::PointStamped minPstamped_local, maxPstamped_local;
//      minPstamped_local.point.x = minP.x;
//      minPstamped_local.point.y = minP.y;
//      minPstamped_local.header = cloud_in_->header;
//      maxPstamped_local.point.x = maxP.x;
//      maxPstamped_local.point.y = maxP.y;
//      maxPstamped_local.header = cloud_in_->header;
//      geometry_msgs::PointStamped minPstamped_global, maxPstamped_global;
////       try
//       {
//         tf_.transformPoint (global_frame_, minPstamped_local, minPstamped_global);
//         tf_.transformPoint (global_frame_, maxPstamped_local, maxPstamped_global);
//         resp.table.table_min.x = minPstamped_global.point.x;
//         resp.table.table_min.y = minPstamped_global.point.y;
//         resp.table.table_max.x = maxPstamped_global.point.x;
//         resp.table.table_max.y = maxPstamped_global.point.y;
//       }
//       catch (tf::TransformException)
//       {
//         ROS_ERROR ("Failed to transform table bounds from frame %s to frame %s", cloud_in_.header.frame_id.c_str (), global_frame_.c_str ());
//         return (false);
//       }
/// ----------------------------------------------------------------------- CONVEX HULL

      // Compute the convex hull
       pmap_.header.stamp = cloud_down_.header.stamp;
       pmap_.header.frame_id = cloud_in_->header.frame_id;
       pmap_.polygons.resize (1);
       cloud_geometry::areas::convexHull2D (cloud_down_, inliers, coeff, pmap_.polygons[0]);
       cloud_geometry::nearest::computeCentroid (cloud_down_, inliers, table.table_center);
       table.table_polygon.header.frame_id = pmap_.header.frame_id;
       table.table_polygon.polygon.points = pmap_.polygons[0].points;
       table_pub_.publish (table);

/// ----------------------------------------------------------------------- TRANSFORM CONVEX HULL
      // Find the object clusters supported by the table
       inliers.clear ();
       findObjectClusters (*cloud_in_, coeff, pmap_.polygons[0], minP, maxP, inliers);

      // Transform into the global frame
//       try
//       {
//         geometry_msgs::PointStamped local, global;
//         local.header = cloud_down_.header;
//         for (unsigned int i = 0; i < pmap_.polygons.size (); i++)
//         {
//           for (unsigned int j = 0; j < pmap_.polygons[i].points.size (); j++)
//           {
//             local.point.x = pmap_.polygons[i].points[j].x;
//             local.point.y = pmap_.polygons[i].points[j].y;
//             tf_.transformPoint (global_frame_, local, global);
//             pmap_.polygons[i].points[j].x = global.point.x;
//             pmap_.polygons[i].points[j].y = global.point.y;
//           }
//         }
//       }
//       catch (tf::TransformException)
//       {
//         ROS_ERROR ("Failed to PolygonalMap from frame %s to frame %s", cloud_down_.header.frame_id.c_str(), global_frame_.c_str());
//         return (false);
//       }

//       resp.table.table = pmap_.polygons[0];
/// ----------------------------------------------------------------------- 


//       ROS_INFO ("Table found. Bounds: [%f, %f] -> [%f, %f]. Number of objects: %d. Total time: %f.",
//                 resp.table.table_min.x, resp.table.table_min.y, resp.table.table_max.x, resp.table.table_max.y, (int)resp.table.objects.size (), (ros::Time::now () - ts).toSec ());

//       // Should only used for debugging purposes (on screen visualization)
//       if (publish_debug_)
//       {
//         // Break the object inliers into clusters in an Euclidean sense
//         vector<vector<int> > objects;
//         cloud_geometry::nearest::extractEuclideanClusters (*cloud_in_, inliers, object_cluster_tolerance_, objects, -1, -1, -1, -1, object_cluster_min_pts_);

//         int total_nr_pts = 0;
//         for (unsigned int i = 0; i < objects.size (); i++)
//           total_nr_pts += objects[i].size ();

//         cloud_annotated_.header = cloud_down_.header;
//         cloud_annotated_.points.resize (total_nr_pts);

//         // Copy all the channels from the original pointcloud
//         cloud_annotated_.channels.resize (cloud_in_->channels.size () + 1);
//         for (unsigned int d = 0; d < cloud_in_->channels.size (); d++)
//         {
//           cloud_annotated_.channels[d].name = cloud_in_->channels[d].name;
//           cloud_annotated_.channels[d].values.resize (total_nr_pts);
//         }
//         cloud_annotated_.channels[cloud_in_->channels.size ()].name = "rgb";
//         cloud_annotated_.channels[cloud_in_->channels.size ()].values.resize (total_nr_pts);

//         // For each object in the set
//         int nr_p = 0;
//         for (unsigned int i = 0; i < objects.size (); i++)
//         {
//           float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));
//           // Get its points
//           for (unsigned int j = 0; j < objects[i].size (); j++)
//           {
//             cloud_annotated_.points[nr_p] = cloud_in_->points.at (objects[i][j]);
//             for (unsigned int d = 0; d < cloud_in_->channels.size (); d++)
//               cloud_annotated_.channels[d].values[nr_p] = cloud_in_->channels[d].values.at (objects[i][j]);
//             cloud_annotated_.channels[cloud_in_->channels.size ()].values[nr_p] = rgb;
//             nr_p++;
//           }
//         }
//         cloud_pub_.publish (cloud_annotated_);
//         pmap_pub_.publish (pmap_);
//       }
      ROS_INFO ("- detectPlanes");
      return (true);
    }
    
    int
      fitSACPlane (sensor_msgs::PointCloud *points, std::vector<int> *indices, std::vector<int> &inliers, std::vector<double> &coeff)
    {
      ROS_INFO ("+ fitSACPlane");
      if ((int)indices->size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (-1);
      }
      ROS_INFO ("1 fitSACPlane");

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (500);
      sac->setProbability (0.99);
      model->setDataSet (points, *indices);

      ROS_INFO ("2 fitSACPlane");
      // Search for the best plane
      if (sac->computeModel ())
      {
      ROS_INFO ("3 fitSACPlane");
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
        {
          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
      ROS_INFO ("3a fitSACPlane");
          inliers.resize (0);
      ROS_INFO ("3b fitSACPlane");
          coeff.resize (0);
          return (0);
        }
      ROS_INFO ("3c fitSACPlane");
        sac->computeCoefficients (coeff);     // Compute the model coefficients
      ROS_INFO ("4 fitSACPlane");
        sac->refineCoefficients (coeff);      // Refine them using least-squares
        model->selectWithinDistance (coeff, sac_distance_threshold_, inliers);

        //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
        //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      ROS_INFO ("5 fitSACPlane");
      }
      ROS_INFO ("- fitSACPlane");
      return (0);
    }

    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      ROS_INFO ("PointCloud message received on %s with %d points. Detecting table..", input_cloud_topic_.c_str (), (int)cloud->points.size ());
      detectPlanes (cloud);
    }

    bool 
      spin ()
    {
      while (ros::ok())
      {
        ros::spinOnce ();
      }
      return true;
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "find_clusters");

  FindClusters n;
  n.spin ();

  return (0);
}

