/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* author: Radu Bogdan Rusu <rusu@cs.tum.edu> */

/**

@mainpage

@htmlinclude manifest.html

@b visualizer_fusion is a driver which subscribes to 5 different topics (see below), gets the data, applies the necessary transformation, 
and visualizes the results on screen using VTK.

<hr>

@section usage Usage
@verbatim
$ visualizer_fusion [standard ROS args]
@endverbatim

@par Example

@verbatim
$ visualizer_fusion
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "cloud_sr"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloud.html">PointCloud</a> : Point cloud data from the SwissRanger.
- @b "cloud_stoc"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloud.html">PointCloud</a> : Point cloud data from the STOC.
- @b "images_sr"/<a href="../../std_msgs/html/classstd__msgs_1_1ImageArray.html">ImageArray</a> : Distance and intensity camera images from the SwissRanger
- @b "images_stoc"/<a href="../../std_msgs/html/classstd__msgs_1_1ImageArray.html">ImageArray</a> : Left, right, and disparity camera images from the STOC.
- @b "image_flir"/<a href="../../std_msgs/html/classstd__msgs_1_1Image.html">Image</a> : Thermal camera image from the FLIR.

 **/

// ROS core
#include "ros/node.h"
#include "ros/time.h"
#include "ros/common.h"

#include "std_msgs/PointCloud.h"
#include "std_msgs/Point32.h"
#include "std_msgs/ImageArray.h"
#include "std_msgs/Image.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

// VTK includes
#include "vtk_tools.h"

using namespace std;

class VisualizerFusion: public ros::node
{
  public:

    // Flags
    bool dump_to_disk_, sr_enable_, stoc_enable_, flir_enable_;

    // VTK stuff
    XtAppContext app_context_;
    vtkRenderer *ren_;
    vtkRenderWindow *ren_win_;
    vtkXRenderWindowInteractor *iren_;
    vtkWGInteractorStyle* style_;
//    vtkInteractorStyleTrackballCamera* style_;

//    vtkTextActor* fps_txt_;
//    vtkFPSCallback* fps_cbk_;

    vtkPolyData *sr_vdata_, *stoc_vdata_;
    vtkPolyDataMapper *sr_vmapper_, *stoc_vmapper_;
    vtkActor *sr_vactor_, *stoc_vactor_;
    vtkCriticalSection *sr_lock_;//, *stoc_lock_;

    // ROS messages
    std_msgs::PointCloud sr_msg_cloud_, stoc_msg_cloud_;
    std_msgs::ImageArray sr_msg_images_, stoc_msg_images_;
    std_msgs::Image flir_msg_image_, stoc_msg_left_image_;

    // TF transformations
//    tf::TransformListener sr_stoc_tfl_;
//    tf::TransformBroadcaster *sr_stoc_tfl_b_;
    
    bool stoc_image_just_left_;

    NEWMAT::Matrix
      getIntrinsic_SR4k ()
    {
      NEWMAT::Matrix mat (3, 3);
      mat
        <<   243.3872401954  <<     0.0000000000  <<    93.3830107632
        <<     0.0000000000  <<   242.8973902804  <<    89.0503222681
        <<     0.0000000000  <<     0.0000000000  <<     1.0000000000
      ;
        // Camera Distortion Coefficients:  <<    -0.8598466633  <<     0.6007425277  <<    -0.0268755414  <<    -0.0162393427
      return (mat);
    }
    
    NEWMAT::Matrix
      getIntrinsic_STOCLeftforSR4k ()
    {
      NEWMAT::Matrix mat (3, 3);
      mat
//        <<   416.3264248449  <<     0.0000000000  <<   325.4263388542
//        <<     0.0000000000  <<   415.4679821276  <<   232.6518187203
//        <<     0.0000000000  <<     0.0000000000  <<     1.0000000000

//        <<   422.477840      <<     0.0000000000  <<   299.101309
//        <<     0.0000000000  <<   421.924602  	  <<   229.911884
//        <<     0.0000000000  <<     0.0000000000  <<     1.0000000000

/* <<    433.6819  <<      0.0000  <<    326.9448
  <<      0.0000  <<    433.3292  <<    208.2214
   <<      0.0000  <<      0.0000  <<      1.0000
      ;*/

          << 429.609702  <<    0.000000       <<         309.444590
          << 0.000000    <<    429.070702     <<         222.472089
          << 0.000000    <<    0.000000       <<         1.000000 
                                              
      ;
      // Camera Distortion Coefficients: <<    -0.0315072078  <<     0.0711552567  <<     0.0095189250  <<     0.0104319110
      return (mat);
    }

    NEWMAT::Matrix
      getIntrinsic_STOCLeftforThermal ()
    {
      NEWMAT::Matrix mat (3, 3);
      mat
        <<   419.6911150447  <<     0.0000000000  <<   357.7189020786
        <<     0.0000000000  <<   417.8570568447  <<   211.5310351584
        <<     0.0000000000  <<     0.0000000000  <<     1.0000000000
      ;
      // Camera Distortion Coefficients: <<     0.0560644870  <<    -0.2416229823  <<    -0.0095165121  <<     0.0338270278
      return (mat);
    }
    NEWMAT::Matrix
      getDistortion_STOCLeftForThermal ()
    {
      NEWMAT::Matrix mat (1, 4);
      mat
       <<      0.0015  <<     -0.0124  <<      0.0043  <<      0.0121
      ;
      return (mat);
    }

    NEWMAT::Matrix
      getIntrinsic_Thermal ()
    {
      NEWMAT::Matrix mat (3, 3);
      mat
        <<   353.1375800333  <<     0.0000000000  <<   152.8744430024
        <<     0.0000000000  <<   372.2732976233  <<   116.4645386707
        <<     0.0000000000  <<     0.0000000000  <<     1.0000000000
      ;
      // Camera Distortion Coefficients:  <<    -0.4499672276  <<     0.3231831656  <<    -0.0038213355  <<    -0.0079903325
      return (mat);
    }
    
    NEWMAT::Matrix
      getExtrinsic_STOCLeftThermal ()
    {
      NEWMAT::Matrix mat (4, 4);
      mat 
        <<   0.9991  <<   0.0196  <<  -0.0382  <<    6.2801
        <<  -0.0180  <<   0.9990  <<   0.0409  <<  -58.7377
        <<   0.0390  <<  -0.0402  <<   0.9984  <<  - 8.7151
        <<   0       <<   0       <<   0       <<    1
      ;
      return (mat);
    }
    
    NEWMAT::Matrix
      getExtrinsic_ThermalSTOCLeft ()
    {
      NEWMAT::Matrix mat (4, 4);
      mat
        <<     0.9999300360  <<    -0.0031469852  <<    -0.0114026163  <<    -0.2611269426
        <<     0.0033616696  <<     0.9998165272  <<     0.0188576552  <<    50.2231430676
        <<     0.0113411794  <<    -0.0188946677  <<     0.9997571551  <<   -14.0321004046
        <<     0.0000000000  <<     0.0000000000  <<     0.0000000000  <<     1.0000000000
      ;
      return (mat);
    }
    
    NEWMAT::Matrix
      getExtrinsic_STOCLeftSR4k ()
    {
      NEWMAT::Matrix mat (4, 4);
      mat
        <<           0.9987  <<           0.0456  <<           0.0248  << -68.9398
        <<          -0.0448  <<           0.9985  <<          -0.0306  << -57.5900
        <<          -0.0262  <<           0.0295  <<           0.9992  <<  43.9617
        <<     0.0000000000  <<     0.0000000000  <<     0.0000000000  <<     1.0000000000
          ;
      return (mat);
    }
    
    NEWMAT::Matrix
      getExtrinsic_SR4kSTOCLeft ()
    {
      NEWMAT::Matrix mat (4, 4);
      mat
/* GOOD        <<     0.9998348686  <<     0.0121139309  <<     0.0135457856  <<  -105.2099177879
        <<    -0.0122397937  <<     0.9998823270  <<     0.0092476829  <<   -46.9201656270
        <<    -0.0134321658  <<    -0.0094119534  <<     0.9998654870  <<    29.2877791604
        <<     0.0000000000  <<     0.0000000000  <<     0.0000000000  <<     1.0000000000
*/

/*good2*/
        <<   0.9998  <<  -0.0116  <<  0.0162  <<  100.9246
        <<   0.0116  <<   0.9999  <<  0.0018  <<   54.6890
        <<  -0.0162  <<  -0.0016  <<  0.9999  <<  -31.0171
        <<   0       <<   0       <<   0      <<    1
      ;
      for (int d = 1; d < 4; d++)
        mat (d, 4) /= 1000;
  
      return (mat);
    }
    
    NEWMAT::Matrix sr_to_stoc_;
    
    VisualizerFusion () : ros::node ("visualizer_fusion"), dump_to_disk_(false), sr_enable_(true),
                                                           stoc_enable_(true), flir_enable_(true),
                                                           sr_vdata_(NULL), 
                                                           stoc_vdata_(NULL), 
                                                           sr_vmapper_(NULL), 
                                                           stoc_vmapper_(NULL), 
                                                           sr_vactor_(NULL),
                                                           stoc_vactor_(NULL),
                                                           stoc_image_just_left_(false)
//                                                           sr_stoc_tfl_ (*this, true, 10000000000ULL, 0)  // node, interpoation, cache_time, extrapolation_distance
    {
      ren_     = vtkRenderer::New ();

//      fps_txt_ = vtkTextActor::New ();
//      fps_cbk_ = vtkFPSCallback::New ();
//      fps_cbk_->SetTextActor (fps_txt_);

      ren_win_ = vtkRenderWindow:: New ();
      ren_win_->SetWindowName ("Visualizer Fusion");
      sr_lock_    = vtkCriticalSection::New ();
//      stoc_lock_  = vtkCriticalSection::New ();

//      sr_stoc_tfl_b_ = new tf::TransformBroadcaster (*this);

//      NEWMAT::Matrix sr_stoc_calib = getExtrinsic_STOCLeftSR4k ();
      sr_to_stoc_ = getExtrinsic_SR4kSTOCLeft ();
    }

    ~VisualizerFusion ()
    {
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    void
      rotateCloud (std_msgs::PointCloud &cloud, NEWMAT::Matrix transform)
    {
      double t1x, t1y, t1z;
      for (unsigned int i = 0; i < cloud.get_pts_size (); i++)
      {
        // Rotate first
        t1x = cloud.pts[i].x * transform(1, 1) + 
              cloud.pts[i].y * transform(1, 2) +
              cloud.pts[i].z * transform(1, 3) +
              transform(1, 4);

        t1y = cloud.pts[i].x * transform(2, 1) + 
              cloud.pts[i].y * transform(2, 2) +
              cloud.pts[i].z * transform(2, 3) +
              transform(2, 4);

        t1z = cloud.pts[i].x * transform(3, 1) + 
              cloud.pts[i].y * transform(3, 2) +
              cloud.pts[i].z * transform(3, 3) +
              transform(3, 4);
        cloud.pts[i].x = t1x;
        cloud.pts[i].y = t1y;
        cloud.pts[i].z = t1z;
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
      printMatrix (NEWMAT::Matrix mat)
    {
      for (int i = 1; i < 5; i++)
      {
        for (int j = 1; j < 5; j++)
          cerr << mat(i, j) << " ";
        cerr << endl;
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Rotate an image with 180 in place
    void
      rotateImage180 (std_msgs::Image &image)
    {
      NEWMAT::Matrix rotation180 (3, 3);
      
      rotation180 << -1  <<  0  <<  int(image.width)
                  <<  0  << -1  << -int(image.height)  
                  <<  0  <<  0  <<  1;
      
      vector<uint8_t> imgdata;
      memcpy (&(imgdata[0]), &(image.data[0]), image.get_data_size ());

      for (unsigned int i = 0; i < image.height; i++)
      {
        for (unsigned int j = 0; j < image.width; j++)
        {          
          // Rotate
          int ni = j * rotation180(1, 1) + 
                   i * rotation180(1, 2) +
                   rotation180(1, 3);
                   
          int nj = j * rotation180(2, 1) + 
                   i * rotation180(2, 2) +
                   rotation180(2, 3);
          image.data[ni * image.width + nj] = imgdata[i * image.width + j];
      }
    }
      
        
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    void
      rotateCloud (std_msgs::PointCloud &cloud)
    {
      // Construct a matrix for rotating the pointcloud with 180 degrees
      for (unsigned int i = 0; i < cloud.get_pts_size (); i++)
      {
        cloud.pts[i].x *= - 1.0;
        cloud.pts[i].y *= - 1.0;
      }
    
      // Transform the pointcloud from the SwissRanger coordinate system to the STOC coordinate system
      rotateCloud (cloud, sr_to_stoc_);
      
  
/*      sr_stoc_tfl_b_->sendTransform (tf::Stamped<tf::Transform> (tf::Transform (
                        tf::Quaternion (pdata->pos.pa, 0, 0),
                        tf::Point(pdata->pos.px, pdata->pos.py, 0.0)
                    ), ros::Time::now (), "SR-STOC-left"));*/
                                                                                                                                      
/*      sr_stoc_tfl_.setTransform (tf::Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(laser_x_offset, 0,0)), 0, "base_laser", "base"));
      
      try
      {
        this->tf.transformPointCloud ("map", cloud, global_cloud);
      }
      catch (tf::Exception& ex)
      {
        ROS_WARN("%s\n", ex.what ());
      }*/
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
      convertROSPointCloudToVTK (std_msgs::PointCloud msg_cloud, vtkPolyData **pdata)
    {
      int nr_pts = msg_cloud.get_pts_size ();
      int dim    = msg_cloud.get_chan_size ();

      // These are all local
      vtkPoints *points;
      vtkUnsignedCharArray *colors;
      vtkCellArray *verts;

      // First time this is called, create the objects
      if (*pdata == NULL)
      {
        *pdata = vtkPolyData::New ();

        points = vtkPoints::New ();
        points->SetDataTypeToFloat ();

        verts  = vtkCellArray::New ();

        colors = vtkUnsignedCharArray::New ();
        ROS_INFO ("[VisualizerFusion::convertROSPointCloudToVTK] Init VTK data structure.");
      }
      // After that, just point back to them
      else
      {
        points = reinterpret_cast<vtkPolyData*>(*pdata)->GetPoints ();
        verts  = reinterpret_cast<vtkPolyData*>(*pdata)->GetVerts ();
        colors = reinterpret_cast<vtkUnsignedCharArray*>(reinterpret_cast<vtkPolyData*>(*pdata)->GetPointData ()->GetScalars ());
      }

      // Re-init structures
      points->SetNumberOfPoints (nr_pts);
      colors->SetNumberOfComponents (dim);
      // It's *extremely* important that SetNumberOfTuples be called *after* SetNumberOfComponents (!)
      colors->SetNumberOfTuples (nr_pts);
      verts->Initialize ();
      verts->InsertNextCell (nr_pts);

      // Copy data
      double p[3];
      for (vtkIdType i = 0; i < nr_pts; i++)
      {
        p[0] = msg_cloud.pts[i].x; p[1] = msg_cloud.pts[i].y; p[2] = msg_cloud.pts[i].z;
        points->SetPoint (i, p);
        verts->InsertCellPoint (i);

        
        for (int d = 0; d < dim; d++)
        {
          colors->SetComponent (i, d, msg_cloud.chan[d].vals[i]);
//          if (msg_cloud.chan[d].name == "r")
//            colors->SetComponent (i, 0, msg_cloud.chan[d].vals[i]);
//          if (msg_cloud.chan[d].name == "g")
//            colors->SetComponent (i, 1, msg_cloud.chan[d].vals[i]);
//          if (msg_cloud.chan[d].name == "b")
//            colors->SetComponent (i, 2, msg_cloud.chan[d].vals[i]);
        }
      }

      reinterpret_cast<vtkPolyData*>(*pdata)->SetPoints (points);
      reinterpret_cast<vtkPolyData*>(*pdata)->SetVerts (verts);
      reinterpret_cast<vtkPolyData*>(*pdata)->GetPointData ()->SetScalars (colors);
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
      getPixelColor (std_msgs::PointCloud &cloud, NEWMAT::Matrix intrinsic_mat, std_msgs::Image image)
    {
      cloud.set_chan_size (3);
      cloud.chan[0].name = "r";
      cloud.chan[1].name = "g";
      cloud.chan[2].name = "b";
      cloud.chan[0].set_vals_size (cloud.get_pts_size ());
      cloud.chan[1].set_vals_size (cloud.get_pts_size ());
      cloud.chan[2].set_vals_size (cloud.get_pts_size ());
      
      double p[3], ptr[3];
      for (unsigned int i = 0; i < cloud.get_pts_size (); i++)
      {
        p[0] = cloud.pts[i].x;
        p[1] = cloud.pts[i].y;
        p[2] = cloud.pts[i].z;
//        p[0] = (cloud.pts[i].x / cloud.pts[i].z);
//        p[1] = (cloud.pts[i].y / cloud.pts[i].z);
//        p[2] = 1.0;
        
        ptr[0] = p[0] * intrinsic_mat(1, 1) + p[1] * intrinsic_mat(1, 2) + p[2] * intrinsic_mat(1, 3);
        ptr[1] = p[0] * intrinsic_mat(2, 1) + p[1] * intrinsic_mat(2, 2) + p[2] * intrinsic_mat(2, 3);
        ptr[2] = p[0] * intrinsic_mat(3, 1) + p[1] * intrinsic_mat(3, 2) + p[2] * intrinsic_mat(3, 3);

//        if (ptr[2] < 1e-5)
//          continue;
        ptr[0] /= ptr[2];
        ptr[1] /= ptr[2];
        
        if ( (ptr[0] > 640) || (ptr[1] > 480) || (ptr[0] < 0) || (ptr[1] < 0));
        else
        {
          int u =  (ptr[0]);
          int v =  (ptr[1]);
//          cerr << u << " " << v << " " << u + v * 640 << endl;
          cloud.chan[0].vals[i] = image.data[ (u + v * 640) * 3 + 0];
          cloud.chan[1].vals[i] = image.data[ (u + v * 640) * 3 + 1];
          cloud.chan[2].vals[i] = image.data[ (u + v * 640) * 3 + 2];
        }
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    void
      getPixelColor (std_msgs::PointCloud &cloud, std_msgs::Image image)
    {
      NEWMAT::Matrix homography (3, 3);
      homography
       <<      2.1402  <<      0.0623  <<    160.3371
        <<      0.2101  <<      1.9134  <<     89.9841
         <<      0.0006  <<      0.0002  <<      1.0000
      ;

      cloud.set_chan_size (4);
      cloud.chan[0].name = "pid";
      cloud.chan[1].name = "r";
      cloud.chan[2].name = "g";
      cloud.chan[3].name = "b";

      cloud.chan[0].set_vals_size (cloud.get_pts_size ());
      cloud.chan[1].set_vals_size (cloud.get_pts_size ());
      cloud.chan[2].set_vals_size (cloud.get_pts_size ());
      cloud.chan[3].set_vals_size (cloud.get_pts_size ());
      
      int img_width = image.width, img_height = image.height, sr_width = 176, sr_height = 144;
      
      std_msgs::Point32 p, q;
      for (unsigned int i = 0; i < cloud.get_pts_size (); i++)
      {
        int pid = cloud.chan[0].vals[i];
        p.y = round (pid / sr_width);
        p.x = round (pid % sr_width);
        p.z = 1;
        
        q.x = p.x * homography(1, 1) + p.y * homography(1, 2) + p.z * homography(1, 3);
        q.y = p.x * homography(2, 1) + p.y * homography(2, 2) + p.z * homography(2, 3);
        q.z = p.x * homography(3, 1) + p.y * homography(3, 2) + p.z * homography(3, 3);
        
        int nv = round (q.x / q.z);
        int nu = round (q.y / q.z);
        
        cloud.chan[0].vals[i] = pid;
        cloud.chan[1].vals[i] = image.data[(nu * img_width + nv) * 3 + 0];
        cloud.chan[2].vals[i] = image.data[(nu * img_width + nv) * 3 + 1];
        cloud.chan[3].vals[i] = image.data[(nu * img_width + nv) * 3 + 2];
        
//        cloud.chan[1].vals[i] = image.data[(p.x * img_width + p.y) * 2 + 1];
//        cloud.chan[2].vals[i] = image.data[(p.x * img_width + p.y) * 2 + 1];
//        cloud.chan[3].vals[i] = image.data[(p.x * img_width + p.y) * 2 + 1];
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    void
      sr_cloud_cb ()
    {
      // Apply the necessary transformations
      
      rotateCloud (sr_msg_cloud_);
      sr_lock_->Lock ();

      if (stoc_image_just_left_ && stoc_msg_left_image_.get_data_size () > 0)
        getPixelColor (sr_msg_cloud_, getIntrinsic_STOCLeftforSR4k (), stoc_msg_left_image_);
      else if (stoc_msg_images_.get_images_size () > 0)
        getPixelColor (sr_msg_cloud_, getIntrinsic_STOCLeftforSR4k (), stoc_msg_images_.images[0]);
//      if (sr_msg_images_.get_images_size () > 0)
//        getPixelColor (sr_msg_cloud_, sr_msg_images_.images[1]);
//      if (stoc_msg_images_.get_images_size () > 0)
//        getPixelColor (sr_msg_cloud_, stoc_msg_images_.images[0]);
      sr_lock_->Unlock ();
      
      // Convert the data to VTK format
      sr_lock_->Lock ();
      convertROSPointCloudToVTK (sr_msg_cloud_, &sr_vdata_);
      sr_vdata_->Update ();
      sr_lock_->Unlock ();
      fprintf (stderr, "[sr_cloud_cb] Received data packet with %d points.\n", sr_vdata_->GetNumberOfPoints ());

      

      // First time we get data, we create the actor and add it to the renderer
      if (sr_vactor_ == NULL)
      {
        ROS_INFO ("[VisualizerFusion::sr_cloud_cb] Init VTK actor.");
        sr_lock_->Lock ();
        sr_vmapper_ = vtkPolyDataMapper::New ();
        sr_vmapper_->SetInput (sr_vdata_);
        sr_vmapper_->SetScalarModeToUsePointData ();
        sr_vmapper_->ScalarVisibilityOn ();

        sr_vactor_ = vtkLODActor::New ();
        reinterpret_cast<vtkLODActor*>(sr_vactor_)->SetNumberOfCloudPoints (sr_vdata_->GetNumberOfPoints () / 10);
        sr_vactor_->SetMapper (sr_vmapper_);
        sr_vactor_->GetProperty ()->SetRepresentationToPoints ();
        sr_vactor_->GetProperty ()->SetPointSize (4);
        sr_vactor_->GetProperty ()->SetInterpolationToFlat ();
        ren_->AddActor (sr_vactor_);
        sr_lock_->Unlock ();
      }
      else
      {
        sr_lock_->Lock ();
        sr_vdata_->Modified ();
        sr_vmapper_->Modified ();
        sr_vactor_->Modified ();
        sr_lock_->Unlock ();
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
      stoc_cloud_cb ()
    {
      // Convert the data to VTK format
      sr_lock_->Lock ();
      convertROSPointCloudToVTK (stoc_msg_cloud_, &stoc_vdata_);
      sr_lock_->Unlock ();
      fprintf (stderr, "[stoc_cloud_cb] Received data packet with %d points.\n", stoc_vdata_->GetNumberOfPoints ());

      // First time we get data, we create the actor and add it to the renderer
      if (stoc_vactor_ == NULL)
      {
        sr_lock_->Lock ();
        ROS_INFO ("[VisualizerFusion::stoc_cloud_cb] Init VTK actor.");
        stoc_vmapper_ = vtkPolyDataMapper::New ();
        stoc_vmapper_->SetInput (stoc_vdata_);
        stoc_vmapper_->SetScalarModeToUsePointData ();
        stoc_vmapper_->ScalarVisibilityOn ();

        stoc_vactor_ = vtkLODActor::New ();
        reinterpret_cast<vtkLODActor*>(stoc_vactor_)->SetNumberOfCloudPoints (stoc_vdata_->GetNumberOfPoints () / 10);
        stoc_vactor_->SetMapper (stoc_vmapper_);
        stoc_vactor_->GetProperty ()->SetRepresentationToPoints ();
        stoc_vactor_->GetProperty ()->SetInterpolationToFlat ();
        ren_->AddActor (stoc_vactor_);
        sr_lock_->Unlock ();
      }
      else
      {
        sr_lock_->Lock ();
        stoc_vdata_->Modified ();
        stoc_vmapper_->Modified ();
        stoc_vactor_->Modified ();
        sr_lock_->Unlock ();
      }
    }
    
    // We don't bother with these callbacks for now
    void stoc_images_cb () {}
    void sr_images_cb ()   {}
    void flir_image_cb ()  {}

    ////////////////////////////////////////////////////////////////////////////////
    void
      init (int width, int height)
    {
      // VTK related init
      ren_->SetBackground (1.0, 1.0, 1.0);
//      ren_->AddObserver (vtkCommand::EndEvent, fps_cbk_);
//      fps_txt_->GetProperty ()->SetColor (0.0, 0.0, 1.0);
//      ren_->AddActor (fps_txt_);

      ren_win_->SetSize (width, height);
      ren_win_->AddRenderer (ren_);

//      vtkCriticalSection* lock1 = vtkCriticalSection::New ();
//      vtkCriticalSection* lock2 = vtkCriticalSection::New ();

      style_ = vtkWGInteractorStyle::New ();

//      style_ = vtkInteractorStyleTrackballCamera::New ();
      style_->ren_win_ = ren_win_;
//      style_->lock_timer_ = sr_lock_;//lock1;
//      style_->lock_char_  = sr_lock_;//;lock2;
//      style_->UseTimersOn ();

      iren_ = vtkXRenderWindowInteractor::New ();
      iren_->SetRenderWindow (ren_win_);
      iren_->Initialize (app_context_);
      iren_->SetInteractorStyle (style_);

      if (flir_enable_)
      {
        subscribe ("image_flir", flir_msg_image_, &VisualizerFusion::flir_image_cb, 1);
        
      }
      
      if (stoc_enable_)
      {
//        subscribe ("cloud_stoc", stoc_msg_cloud_, &VisualizerFusion::stoc_cloud_cb, 1);
        if (stoc_image_just_left_)
          subscribe ("image_stoc_left", stoc_msg_left_image_, &VisualizerFusion::stoc_images_cb, 1);
        else
          subscribe ("images_stoc", stoc_msg_images_, &VisualizerFusion::stoc_images_cb, 1);
      }
      
      if (sr_enable_)
      {
        subscribe ("cloud_sr", sr_msg_cloud_, &VisualizerFusion::sr_cloud_cb, 1);
//        subscribe ("images_sr", sr_msg_images_, &VisualizerFusion::sr_images_cb, 1);
      }
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  XtToolkitInitialize ();
  
  // Here is where we create our ROS node
  VisualizerFusion f;
  f.app_context_ = XtCreateApplicationContext ();
  f.init (1000, 800);

  // Initialize the VTK interactor and start the ROS node
  while (f.ok ())
  {
    usleep (100000);
    f.sr_lock_->Lock ();
    if (XtAppPending (f.app_context_))
      XtAppProcessEvent (f.app_context_, XtIMAll);
                        
    f.ren_win_->Render ();
    f.sr_lock_->Unlock ();
  }
  
//  f.iren_->Initialize ();
//  f.iren_->CreateRepeatingTimer (10);
//  f.iren_->Start ();

  ros::fini ();

  return (0);
}
/* ]--- */
