// A simple program that computes the square root of a number
//#define  EIGEN_DONT_PARALLELIZE
#include <iostream>
#include <algorithm>    // std::max

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#include <Eigen/Geometry>


#define NB_JOINT 3

#include "./hammer/hammer.hpp"

#include "./arm_hori.hpp"

using namespace hammer;


// Definition of the parameters
struct Params{
  struct hammer{
    HMR_PARAM(bool, stats_enabled, true);
    HMR_PARAM_STRING(name, "LL_hammer");
  };
  struct nearesttarget{
    HMR_PARAM_VECTOR(double, proj, 0,0,0, 0,0,0,0,0,0,1,1,0 );
  };
  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 20);
  };
  struct tolerance{
    HMR_PARAM(double, tol, 0.01);
  };

  
  
  
  struct distance {
    HMR_PARAM(double, offset, 1);
    HMR_PARAM(double, rate, 1);
    HMR_PARAM(double, diff, 0.5);
    
  };
  
  struct pseudomax : public selector::defaults::pseudomax{};

  struct kernel_maternfivehalves {
    /// @ingroup kernel_defaults
    BO_PARAM(double, sigma_sq, 1);
    /// @ingroup kernel_defaults
    BO_PARAM(double, l, 0.3);
  };
  struct gp {
    HMR_PARAM(double, noise, 1e-2);
    HMR_PARAM(int, hp_period, 0);
    HMR_PARAM(bool, transition, true);

  };

}; 
struct Params2:public Params{
  struct hammer{
    HMR_PARAM(bool, stats_enabled, true);
    HMR_PARAM_STRING(name, "HL_hammer");
  };
  struct nearesttarget{
    HMR_PARAM_VECTOR(double, proj, 1,1,1);
  };
  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 1);
  };

  struct kernel_maternfivehalfs {
    /// @ingroup kernel_defaults
    BO_PARAM(double, sigma_sq, 1);
    /// @ingroup kernel_defaults
    BO_PARAM(double, l, 0.1);
  };


  struct gp {
    HMR_PARAM(double, noise, 1e-1);
    HMR_PARAM(int, hp_period, 0);
    HMR_PARAM(bool, transition, false);

  };

}; 


struct Params3:public Params2{
  struct hammer{
    HMR_PARAM(bool, stats_enabled, true);
    HMR_PARAM_STRING(name, "HHL_hammer");
  };

  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 100);
  };


}; 




struct Extension{
  Eigen::VectorXd operator()(Eigen::VectorXd x){
    Eigen::VectorXd Y(3*x.rows());//+x.rows()*x.rows());

    Y.head(x.rows())=x;
    Y.segment(x.rows(), x.rows())=x.array().sin();
    Y.segment(2*x.rows(), x.rows())=x.array().cos();   
  /*for(int i=0;i<x.rows();i++){
      Y.segment((2+i)*x.rows(),x.rows())=x*x(i);
      }*/
    return Y;
  } 
};

template<int N>
class MonoJoint{
public:
  MonoJoint(){}
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){

    Eigen::VectorXd action=Eigen::VectorXd::Zero(NB_JOINT);
    action[0]=atan2(current[1+3*1],current[3*1]);
    for(int i=1;i<NB_JOINT;i++){

      action[i]=atan2(current[1+3*(i+1)]-current[1+3*(i)],current[3*(i+1)]-current[3*(i)])-action.segment(0,i).sum();
      if(action[i]>M_PI)
	action[i]-=2*M_PI;

      if(action[i]<-M_PI)
	action[i]+=2*M_PI;
}

    Eigen::Vector3d Ptarget=target.tail(3)-current.segment(N*3,3);
    Eigen::Vector3d Peff=current.tail(3)-current.segment(N*3,3);
    Eigen::Vector3d cross=Peff.cross(Ptarget);



    
    action(N)+=atan2(cross(2),Ptarget.transpose()*Peff);
    return action;
  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

};


template<int N>
class MonoVel{
public:
  MonoVel(double val ):_val(val){}
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    Eigen::VectorXd action=Eigen::VectorXd::Zero(NB_JOINT);
    action(N)=_val;
    return action;
  }
  double _val;
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

};


class DirectTarget{
public:
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    Eigen::VectorXd action=Eigen::VectorXd::Zero(3*(NB_JOINT+1));
    action.tail(3)=target;
    return action;
  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

};

template <typename Params>
struct ActionForwarding {
  ActionForwarding(size_t dim_out = 1) : _dim_out(dim_out) {}

  template <typename GP>
  Eigen::VectorXd operator()(const Eigen::VectorXd& v, const GP&) const
  {
    return v.tail(_dim_out);
  }

protected:
  size_t _dim_out;
};




int main (int argc, char *argv[])
{
  std::srand(std::time(NULL));
  srand(time(NULL));
  
  // Instantiation of the architecture

  typedef Hammer<Params,scorefun<scoreupdator::NearestTarget<Params> >, selectfun<selector::ScoreProportionate<Params> >, stopcrit<boost::fusion::vector<stop::MaxIterations<Params>,stop::Tolerance<Params>  > > > hmr1_t;
  typedef Hammer<Params2,scorefun<scoreupdator::NearestTarget<Params2> >, selectfun<selector::NonDominated<Params2> >, stopcrit<stop::MaxIterations<Params2> > > hmr2_t;
  typedef Hammer<Params3,scorefun<scoreupdator::NearestTarget<Params3> >, selectfun<selector::PseudoMax<Params3> >, stopcrit<stop::MaxIterations<Params3> > > hmr3_t;
  typedef boost::fusion::vector<robot::Arm, hmr3_t,hmr1_t,hmr2_t> compo_t;
  Hierarchy<4,compo_t> hierarchy;

  MonoVel<0> V01(1);
  hierarchy.bindInverseModel<1>(V01);
  MonoVel<0> V02(0.5);
  hierarchy.bindInverseModel<1>(V02);
  MonoVel<0> V03(0.2);
  hierarchy.bindInverseModel<1>(V03);
  MonoVel<0> V04(0.1);
  hierarchy.bindInverseModel<1>(V04);
  MonoVel<0> V05(0);
  hierarchy.bindInverseModel<1>(V05);
  MonoVel<0> V06(-0.1);
  hierarchy.bindInverseModel<1>(V06);
  MonoVel<0> V07(-0.2);
  hierarchy.bindInverseModel<1>(V07);
  MonoVel<0> V08(-0.5);
  hierarchy.bindInverseModel<1>(V08);
  MonoVel<0> V09(-1);
  hierarchy.bindInverseModel<1>(V09);


  MonoVel<1> V11(1);
  hierarchy.bindInverseModel<1>(V11);
  MonoVel<1> V12(0.5);
  hierarchy.bindInverseModel<1>(V12);
  MonoVel<1> V13(0.2);
  hierarchy.bindInverseModel<1>(V13);
  MonoVel<1> V14(0.1);
  hierarchy.bindInverseModel<1>(V14);
  MonoVel<1> V15(0);
  hierarchy.bindInverseModel<1>(V15);
  MonoVel<1> V16(-0.1);
  hierarchy.bindInverseModel<1>(V16);
  MonoVel<1> V17(-0.2);
  hierarchy.bindInverseModel<1>(V17);
  MonoVel<1> V18(-0.5);
  hierarchy.bindInverseModel<1>(V18);
  MonoVel<1> V19(-1);
  hierarchy.bindInverseModel<1>(V19);

  
  MonoVel<2> V21(1);
  hierarchy.bindInverseModel<1>(V21);
  MonoVel<2> V22(0.5);
  hierarchy.bindInverseModel<1>(V22);
  MonoVel<2> V23(0.2);
  hierarchy.bindInverseModel<1>(V23);
  MonoVel<2> V24(0.1);
  hierarchy.bindInverseModel<1>(V24);
  MonoVel<2> V25(0);
  hierarchy.bindInverseModel<1>(V25);
  MonoVel<2> V26(-0.1);
  hierarchy.bindInverseModel<1>(V26);
  MonoVel<2> V27(-0.2);
  hierarchy.bindInverseModel<1>(V27);
  MonoVel<2> V28(-0.5);
  hierarchy.bindInverseModel<1>(V28);
  MonoVel<2> V29(-1);
  hierarchy.bindInverseModel<1>(V29);

  forwardModels::LinearLeastSquare<> LLS1;  
  hierarchy.bindForwardModel<1>(LLS1);
  
  MonoJoint<0> M0;
  hierarchy.bindInverseModel<2>(M0);

  MonoJoint<1> M1;
  hierarchy.bindInverseModel<2>(M1);

  MonoJoint<2> M2;
  hierarchy.bindInverseModel<2>(M2);

  DirectTarget IMTarget;
  hierarchy.bindInverseModel<3>(IMTarget);
  
  



  typedef limbo::kernel::MaternFiveHalves<Params> Kernel_t;
  forwardModels::GP<Params, Kernel_t, limbo::mean::NullFunction<Params>  > gp1((NB_JOINT+2)*3,(NB_JOINT+1)*3);
  hierarchy.bindForwardModel<2>(gp1);

  //forwardModels::LinearLeastSquare<Extension> LLS2;
  //hierarchy.bindForwardModel<2>(LLS2);

  
  forwardModels::LinearLeastSquare<> LLS3;
  typedef limbo::kernel::MaternFiveHalves<Params2> Kernel2_t;
  forwardModels::GP<Params2, Kernel2_t, limbo::mean::NullFunction<Params2>  > gp2(3,3);
  hierarchy.bindForwardModel<3>(LLS3);



  /*  forwardModels::GP<Params3, Kernel2_t, ActionForwarding<Params3>  > gp3(3,3);
  hierarchy.bindForwardModel<3>(gp3);
  FMOptimizer<forwardModels::GP<Params3, Kernel2_t, ActionForwarding<Params3>  > > fmopt(gp3,3);*/
  //hierarchy.bindInverseModel<3>(fmopt);
  
  hierarchy.printStructure();  
  
  //  Eigen::VectorXd target=Eigen::VectorXd::Zero(3*(NB_JOINT+1));
  Eigen::VectorXd target=Eigen::VectorXd::Zero(3*(1));
  target.tail(3)<<-0.2,-0.1,0;


  //hierarchy.run(target);
  //return 0;


  std::ofstream file("res.dat");
  
  for(int i=0;i<150;i++){
    float f=(float)std::rand()/RAND_MAX;
    target.tail(3)<<0.2 +0.1*cos(2*M_PI* f) , 0.1*sin(2*M_PI*f),0;
    hierarchy.run(target);
    file<<i<<" "<<target.tail(3).transpose()<<"  "<<hierarchy.system().getCurrentState(2).transpose()<<std::endl;
  }
  
  return 0;
}
