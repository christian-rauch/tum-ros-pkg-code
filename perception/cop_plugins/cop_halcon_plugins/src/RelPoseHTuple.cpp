#include "RelPoseHTuple.h"
#include "RelPoseFactory.h"

/*RelPose::RelPose(Halcon::HTuple& relPose, Halcon::HTuple& covariance, RelPose* pose) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
  Update(relPose, covariance, pose);
  //cout << "RelPoseConstructor Matrix:\n" << GetMatrix() <<std::endl;
}*/


using namespace cop;

RelPose* RelPoseHTuple::FRelPose(Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation, unsigned long id)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  if(id != -1)
  {
    RelPose* pose = RelPoseFactory::FRelPose(id);
    if(pose != NULL)
      Update(pose, poseDesc, covariance, relation);
    return pose;
  }
  return RelPoseFactory::GetRelPoseIndex(SetRelPose(new RelPose(poseDesc, covariance, relation)));
#else /*NO_LO_SERVICE_AVAILABLE*/
  if(id != -1 && relation != NULL)
  {
    RelPose* pose_own = RelPoseFactory::FRelPose(id);
    if(pose_own != NULL && pose_own->m_parentID == relation->m_uniqueID)
    {
      Update(pose_own, poseDesc, covariance, relation);
      RelPoseFactory::s_loService->NotifyPoseUpdate(pose_own, true);
      return pose_own;
    }
    else
    {
      delete pose_own;
      Matrix m(4,4);
      Matrix d(6,6);
      if(!RelPoseHTuple::TupleToMat(poseDesc, covariance, m, d))
        printf("Error converting tuple to matrix in FRelPose\n");
      return RelPoseFactory::s_loService->CreateNewPose(relation, &m, &d);
    }

  }
  else
  {
    Matrix m(4,4);
    Matrix d(6,6);
    if(!RelPoseHTuple::TupleToMat(poseDesc, covariance, m, d))
      printf("Error converting tuple to matrix in FRelPose\n");
    return RelPoseFactory::s_loService->CreateNewPose(relation, &m, &d);
  }
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

bool RelPoseHTuple::TupleToMat(Halcon::HTuple& poseDesc, Halcon::HTuple&  covariance, Matrix &m, Matrix &d)
{
      if(poseDesc.Num() > 6)
      {
        Halcon::HTuple hommat;
        if(poseDesc.Num() == 12)
            hommat = poseDesc;
        else
           Halcon::pose_to_hom_mat3d(poseDesc, &hommat);
        m << hommat[0].D() << hommat[1].D() << hommat[2].D() << hommat[3].D()
          << hommat[4].D() << hommat[5].D() << hommat[6].D() << hommat[7].D()
          << hommat[8].D() << hommat[9].D() <<hommat[10].D() <<hommat[11 ].D()
          << 0 << 0 <<0 << 1;
        d.element(0,0)=(covariance.Num() > 0 ? covariance[0].D() : 0.001);
        d.element(1,1)=(covariance.Num() > 1 ? covariance[1].D() : 0.001);
        d.element(2,2)=(covariance.Num() > 2 ? covariance[2].D() : 0.001);
        d.element(3,3)=(covariance.Num() > 3 ? ((covariance[3].D() / 180) * M_PI) : 0.001);
        d.element(4,4)=(covariance.Num() > 4 ? ((covariance[4].D() / 180) * M_PI) : 0.001);
        d.element(5,5)=(covariance.Num() > 5 ? ((covariance[5].D() / 180) * M_PI) : 0.001);
        d.element(0,1) = 0.0; d.element(0,2) = 0.0;d.element(0,3) = 0.0; d.element(0,4) = 0.0;d.element(0,5) = 0.0;
        d.element(1,0) = 0.0; d.element(1,2) = 0.0;d.element(1,3) = 0.0; d.element(1,4) = 0.0;d.element(1,5) = 0.0;
        d.element(2,0) = 0.0; d.element(2,1) = 0.0;d.element(2,3) = 0.0; d.element(2,4) = 0.0;d.element(2,5) = 0.0;
        d.element(3,0) = 0.0; d.element(3,1) = 0.0;d.element(3,2) = 0.0; d.element(3,4) = 0.0;d.element(3,5) = 0.0;
        d.element(4,0) = 0.0; d.element(4,1) = 0.0;d.element(4,2) = 0.0; d.element(4,3) = 0.0;d.element(4,5) = 0.0;
        d.element(5,0) = 0.0; d.element(5,1) = 0.0;d.element(5,2) = 0.0; d.element(5,3) = 0.0;d.element(5,4) = 0.0;
        return true;
      }
      return false;
}

void RelPoseHTuple::Update(RelPose* pose, Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  pose->m_relation = relation;
#endif /*NO_LO_SERVICE_AVAILABLE*/
  Halcon::HTuple hommat;
  if(poseDesc.Num() > 6)
  {
    Matrix m(4,4);
    Matrix d(6,6);
    TupleToMat(poseDesc, covariance, m, d);
    pose->Set(m, d);
  }
}


#ifdef _DEBUG
void RelPoseHTuple::Print(RelPose* pose_in)
{
  Halcon::HTuple pose;
  printf("Printing Pose id: %ld parent id %ld\n", pose_in->m_uniqueID, pose_in->m_parentID);
  GetPose(pose_in, &pose);
  for(int i = 0; i < pose.Num() -1; i++)
  {
    printf("%f, ", pose[i].D());
  }
  printf("%d\n", pose[pose.Num() -1].I());
}
#endif
void RelPoseHTuple::GetHommat(RelPose* pose, Halcon::HTuple* hommat, unsigned long poseRel)
{
  Matrix m(4,4);
  if(poseRel > 0 && (unsigned)poseRel != pose->m_parentID)
  {
      RelPose* pose_rel = RelPoseFactory::GetRelPose(pose->m_uniqueID, poseRel);
      m = pose_rel->GetMatrix(0);
      RelPoseFactory::FreeRelPose(pose_rel);
  }
  else
    m = pose->GetMatrix(0);
  Halcon::tuple_gen_const(12,0,hommat);
  (*hommat)[0] = m.element(0,0);
  (*hommat)[1] = m.element(0,1);
  (*hommat)[2] = m.element(0,2);
  (*hommat)[3] = m.element(0,3);
  (*hommat)[4] = m.element(1,0);
  (*hommat)[5] = m.element(1,1);
  (*hommat)[6] = m.element(1,2);
  (*hommat)[7] = m.element(1,3);
  (*hommat)[8] = m.element(2,0);
  (*hommat)[9] = m.element(2,1);
  (*hommat)[10] = m.element(2,2);
  (*hommat)[11] = m.element(2,3);
}
void RelPoseHTuple::GetPose(RelPose* pose, Halcon::HTuple* poses, unsigned long poseRel)
{
  Halcon::HTuple hommat;
  GetHommat(pose, &hommat, poseRel);
  Halcon::hom_mat3d_to_pose(hommat,poses);
}

