// This controller implements a cartesian impedance (stiffness and damping)
// on top of joint impedance control.
//
// The allowed inputs are:
//
// q_desired, K_cart, D_cart, ft_cart
//
// The cartesian parameters are expressed in the reference frame of the base
// with the reference point flanche.

#include "CartesianImpedance.hh"
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>


#include <kdl/utilities/svd_eigen_HH.hpp>
#include <Eigen/Core>

using namespace KDL;
using namespace Eigen;


// note: K and D are 6-vectors in the FRI structs.
// (those guys 'simplified' the interface)
// Is that bad for us? Well ...
// ... No. We just don't use it :)


CartesianImpedanceControl::CartesianImpedanceControl()
  : q_kdl_(7), jnt2jac_(0), J_kdl_(7), 
    U(6,7), V(6,6), S(7), Sp(7), tmp(7)
{
  chain_.addSegment(Segment(Joint(Joint::None),
                Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0)));
  chain_.addSegment(Segment(Joint(Joint::RotZ),
                 Frame::Identity()));

  jnt2jac_ = new ChainJntToJacSolver(chain_);
}


CartesianImpedanceControl::~CartesianImpedanceControl()
{
  delete jnt2jac_;
}


//! adds torques to cmd.addTorque
//! and replaces cmd.stiffness, cmd.damping and cmd.command
tFriCmdData CartesianImpedanceControl::computeRobotCommand(const tFriMsrData& fri_msr,
        const tFriCmdData& fri_cmd, const CartesianImpedance& imp, const float* cmdpos)
{
  for(int i=0; i < 7; i++)
  {
    q_last_[i]  = fri_msr.data.msrJntPos[i];

    q_kdl_(i) = fri_msr.data.msrJntPos[i];
  }

  // define matrix expressions for most of the matrices we use.
  PlainObjectBase<Matrix<float, 7, 1> > q_des = Matrix<float, 7, 1>::Map(&(fri_cmd.cmd.jntPos[0]));
  PlainObjectBase<Matrix<float, 7, 1> > q_cmd = Matrix<float, 7, 1>::Map(&(cmdpos[0]));
  PlainObjectBase<Matrix<float, 7, 1> > q     = Matrix<float, 7, 1>::Map(&(fri_msr.data.msrJntPos[0]));
  PlainObjectBase<Matrix<float, 7, 7> > M     = Matrix<float, 7, 7, RowMajor>::Map(&(fri_msr.data.massMatrix[0]));

  PlainObjectBase<Matrix<float, 6, 6> > D  = Matrix<float, 6, 6, RowMajor>::Map(&(imp.D[0]));
  PlainObjectBase<Matrix<float, 6, 6> > K  = Matrix<float, 6, 6, RowMajor>::Map(&(imp.K[0]));
  PlainObjectBase<Matrix<float, 6, 1> > ft = Matrix<float, 6, 1>::Map(&(imp.ft[0]));

  // compute Jacobian
  jnt2jac_->JntToJac(q_kdl_, J_kdl_);
  // Eigen requires us to explicitly cast this data type, sorry...
  PlainObjectBase<Eigen::Matrix<float,6,7> > J = J_kdl_.data.cast<float>();

  // since no reliable information on the
  // 'normalization' of the damping was to be found,
  // we stick with the joint damping controllers only.
  Matrix<float, 7, 1> damping = (J.transpose()*D*J).diagonal();

  // Note: we are ignoring the term (dJ^T/dq^T)*K*deltax.
  // Assuming that J does not change much in one cartesian cycle,
  // we spare ourselves the hassle of deriving J once more.
  Matrix<float, 7, 1> stiffness = (J.transpose()*K*J).diagonal();

  // this value can use the full matrix K, which the joint impedance controllers can't
  Matrix<float, 7, 1> tau_add = J.transpose() * (K*J*(q_des - q) + ft) - stiffness.asDiagonal()*(q_cmd - q);


  // null space stiffness, commented, requires testing
  //pinv(J_kdl_.data, Jinv);

  //Matrix<float, 7, 1> K_null;
  //float s = imp.K_null;
  //K_null << s, s, s, s, s, s, s;

  //TODO: compute correct mass matrix
  //MatrixXf tau_null = Jinv*J*K_null.asDiagonal()*(q_des - q);

  //tau_add += tau_null;


  tFriCmdData fri_cmd_cart=fri_cmd;
  for(int i=0; i < 7; i++)
  {
    fri_cmd_cart.cmd.jntPos[i]       = cmdpos[i];
    fri_cmd_cart.cmd.addJntTrq[i]    = fri_cmd.cmd.addJntTrq[i] + tau_add(i);
    fri_cmd_cart.cmd.jntStiffness[i] = stiffness(i);
    fri_cmd_cart.cmd.jntDamping[i]   = damping(i);
  }

  return fri_cmd_cart;
}


template <typename T1, typename T2>
void CartesianImpedanceControl::pinv(Eigen::DenseBase<T1>& J, Eigen::DenseBase<T2>& Jinv, double eps)
{
  svd_eigen_HH(J, U, S, V, tmp);

  for(int i=0; i < 7; ++i)
      Sp(i) = (fabs(S(i)) > eps) ? 1.0 / S(i) : 0.0;

  Jinv = (V * Sp.asDiagonal() * U.transpose()).cast<typename T2::Scalar>();
}

