#ifndef ROBOT_ARM_HORI_HPP
#define ROBOT_ARM_HORI_HPP

#define NB_JOINT 3
#include <iostream>
#include <Eigen/Core>

namespace robot{


class Arm{
public:

typedef Eigen::VectorXd state_t;

Arm():_state(Eigen::VectorXd::Zero((NB_JOINT+1)*3)),
      _joint(Eigen::VectorXd::Zero(NB_JOINT)),
      _joint_vel(Eigen::VectorXd::Zero(NB_JOINT))
{
_pos=forward_model(_joint);
}
  
  Eigen::VectorXd getCurrentState(int level=0)const {
if(level==0)
  return _joint;
if(level==1)
  return _state;

return _pos;


}

Eigen::VectorXd operator()(Eigen::VectorXd v){

_joint_vel=v/20;
_joint+=_joint_vel;
_pos=forward_model(_joint);

return _joint;
}
      
      
      Eigen::Vector3d forward_model(Eigen::VectorXd a);
static double max_length(){return p1_height+p2_height+p3_height+p4_height+p5_height+p6_height+p7_height+p8_height;}
protected:

Eigen::VectorXd _state;
Eigen::VectorXd _pos;
Eigen::VectorXd _joint;
Eigen::VectorXd _joint_vel;

        static constexpr  double p1_height = 0.0875;
        static constexpr  double p2_height = 0.078;
        static constexpr  double p3_height = 0.072;
        static constexpr  double p4_height = 0.072;
        static constexpr  double p5_height = 0.071;
        static constexpr  double p6_height = 0.068;
        static constexpr  double p7_height = 0.068;
        static constexpr  double p8_height = 0.065;


    };
}

#endif
