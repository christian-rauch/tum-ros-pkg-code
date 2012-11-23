#ifndef CARTESIAN_IMPEDANCE_HH
#define CARTESIAN_IMPEDANCE_HH

// This controller implements a cartesian impedance (stiffness and damping)
// on top of joint impedance control.
//
// The allowed inputs are:
//
// q_desired, K_cart, D_cart, ft_cart
//
// The cartesian parameters are expressed in the reference frame of the base
// with the reference point flanche.

#include "FRIData.hh"
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <friComm.h>  // KUKA proprietary header


class CartesianImpedanceControl
{
public:

  CartesianImpedanceControl();
  ~CartesianImpedanceControl();

  //! adds torques to cmd.addTorque
  //! and replaces cmd.stiffness, cmd.damping and cmd.command
  tFriCmdData computeRobotCommand(const tFriMsrData& state, const tFriCmdData& cmd,
                                  const CartesianImpedance& imp, const float* cmdpos);

private:
  float q_last_[7];

  KDL::JntArray q_kdl_;
  KDL::Chain chain_;
  KDL::ChainJntToJacSolver* jnt2jac_;
  KDL::Jacobian J_kdl_;

  // helper matrices for Jacobian inversion

  Eigen::MatrixXd U, V;
  Eigen::Matrix<float, 7, 6> Jinv;
  Eigen::VectorXd S, Sp, tmp;

  template <typename T1, typename T2>
  void pinv(Eigen::DenseBase<T1>& J, Eigen::DenseBase<T2>& Jinv, double eps=1e-13);
};

#endif // CARTESIAN_IMPEDANCE_HH

