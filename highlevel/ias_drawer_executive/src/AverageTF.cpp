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


#include <ias_drawer_executive/AverageTF.h>

tf::Stamped<tf::Pose> AverageTF::getMarkerTransform(const char tf_frame[], double numSamples)
{
  tf::Stamped<tf::Pose> marker;
  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Rate rate(100.0);
  int count = 0;

  double lastTime = 0;
  tf::Stamped<tf::Pose> transSum;
  int i = 0;
  geometry_msgs::Quaternion rot[500];
  geometry_msgs::Vector3 trans[500];

  int numh = 0;
  int numv = 0;


  while (count < numSamples  && ros::ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_link", tf_frame,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
    }

    double actTime = transform.stamp_.toSec();

    //ROS_INFO("I see a ARToolkit Marker. %f", transform.stamp_.toSec());

    if (actTime!=lastTime) {
      lastTime=actTime;
     // ROS_INFO("NEW MARKER POSITION %i" , count);
      marker.frame_id_ = transform.frame_id_;
      marker.setOrigin(transform.getOrigin());
      marker.setRotation(transform.getRotation());
      marker.stamp_ = transform.stamp_;

      transform.setRotation(transform.getRotation().normalize());
      trans[count].x = transform.getOrigin().x();
      trans[count].y = transform.getOrigin().y();
      trans[count].z = transform.getOrigin().z();
      rot[count].x = transform.getRotation().x();
      rot[count].y = transform.getRotation().y();
      rot[count].z = transform.getRotation().z();
      rot[count].w = transform.getRotation().w();
      count++;

      double meanZ = 0;
      for (i = 0; i < count; i++)
      meanZ += trans[i].z;
      meanZ /= (double)count;

      double deviation=0;
      for (i = 0; i < count; i++)
         deviation+= (trans[i].z - meanZ) * (trans[i].z - meanZ);
      deviation/= (double)count;

      printf("\a");

      ROS_INFO("new marker %i STD DEVIATION IN Z %f", count, sqrtf(deviation));

      ROS_INFO("CURRENT MARKER %f %f %f ",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      ROS_INFO("CURRENT MARKER OR %f %f %f %f", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());


      tf::Stamped<tf::Pose> decid;
      decid.frame_id_ = "none";
      decid.stamp_ = ros::Time();
      decid.setOrigin(btVector3(1,0,0));
      decid.setRotation(btQuaternion(0,0,0,1));
      btTransform classify = transform * decid;

      tf::Stamped<tf::Pose> decid2;
      decid2.frame_id_ = "none";
      decid2.stamp_ = ros::Time();
      decid2.setOrigin(btVector3(0,1,0));
      decid2.setRotation(btQuaternion(0,0,0,1));

      btTransform classify2 = transform * decid2;

      ROS_INFO("DECID :c  %f c1 %f m %f", classify.getOrigin().z(), classify2.getOrigin().z(), transform.getOrigin().z());

      if (classify.getOrigin().z() < classify2.getOrigin().z()) {
          numh++;
          ROS_INFO("HORIZONTAL %i", numh);
      } else {
          numv++;
          ROS_INFO("VERTICAL %i", numv);
      }

      //tf::Stamped<tf::Pose> rot;
      //rot.setRotation(btQuaternion(0,0,-M_PI/2));

      //btQuaternion myq(transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
      //btQuaternion qy = myq * btQuaternion(0,M_PI / 2,0);;
      //ROS_INFO("ROT Y %f %f %f %f", qy.x(),qy.y(),qy.z(),qy.w());

     //btVector3 euler;
     //QuaternionToEuler(btQuaternion(rot[count].x,rot[count].y,rot[count].z,rot[count].w).normalize(), euler);
     //QuaternionToEuler(transform.getRotation().normalize(), euler);

     //ROS_INFO("EULER %f %f %f", euler.x(),euler.y(),euler.z());
    }

    rate.sleep();
  }

  //remove outliers
  double meanZ = 0;
  for (i = 0; i < numSamples; i++)
	  meanZ += trans[i].z;
  meanZ /= numSamples;

  double deviation=0;
  for (i = 0; i < numSamples; i++)
	  deviation+= (trans[i].z - meanZ) * (trans[i].z - meanZ);
  deviation /= numSamples;

  ROS_INFO("STD DEVIATION IN Z %f", sqrtf(deviation));

  geometry_msgs::Quaternion rotMean;
  geometry_msgs::Vector3 transMean;

  transMean.x = 0;
  transMean.y = 0;
  transMean.z = 0;
  rotMean.x = 0;
  rotMean.y = 0;
  rotMean.z = 0;
  rotMean.w = 0; // would otherwise be 1

  double numGoodSamples = 0;
  for (i = 0; i < numSamples; i++)
     if ((trans[i].z - meanZ) * (trans[i].z - meanZ) < deviation) {
		 transMean.x += trans[i].x;
		 transMean.y += trans[i].y;
		 transMean.z += trans[i].z;
		 rotMean.x += rot[i].x;
		 rotMean.y += rot[i].y;
		 rotMean.z += rot[i].z;
		 rotMean.w += rot[i].w;
		 numGoodSamples+=1;
	  }

  ROS_INFO("Number of accepted samples %f", numGoodSamples);

  transMean.x /= numGoodSamples;
  transMean.y /= numGoodSamples;
  transMean.z /= numGoodSamples;
  rotMean.x /= numGoodSamples;
  rotMean.y /= numGoodSamples;
  rotMean.z /= numGoodSamples;
  rotMean.w /= numGoodSamples;


  if (numh > numv) {
     rotMean.x = -.711;
     rotMean.y = -.008;
     rotMean.z = .005;
     rotMean.w = .703;
  } else {
     rotMean.x = 0;
     rotMean.y = 0;
     rotMean.z = 0;
     rotMean.w = 1;
  }

  marker.setOrigin(btVector3(transMean.x,transMean.y,transMean.z));
  marker.setRotation(btQuaternion(rotMean.x,rotMean.y,rotMean.z,rotMean.w).normalize());
  return marker;
}
