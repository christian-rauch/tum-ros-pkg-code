#ifndef RELPOSEHTUPLE_H
#define RELPOSEHTUPLE_H

#include "RelPose.h"
#include "cpp/HalconCpp.h"

namespace cop
{
  class RelPoseHTuple
  {
  public:
    static void Update(RelPose* pose, Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation);
    static bool TupleToMat(Halcon::HTuple& poseDesc, Halcon::HTuple&  covariance, Matrix &m, Matrix &d);
    static void GetPose(RelPose* posein, Halcon::HTuple* pose, unsigned long poseRel = 0);
    static void GetHommat(RelPose* pose, Halcon::HTuple* hommat, unsigned long poseRel) ;
    static void Print(RelPose* pose);

    static RelPose* FRelPose(Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation, unsigned long id = -1);

  };
}
#endif /*RELPOSEHTUPLE_H*/
