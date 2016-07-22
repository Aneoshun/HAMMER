// A simple program that computes the square root of a number
//#define  EIGEN_DONT_PARALLELIZE
#include <iostream>
#include <algorithm>    // std::max

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#include "./hammer/hammer.hpp"

#include "./vehicle.hpp"

using namespace hammer;

// Definition of the parameters
struct Params{
  struct kernel_squared_exp_ard {
    BO_PARAM(int, k, 0); //equivalent to the standard exp ARD
    BO_PARAM(double, sigma_sq, 1);
  };
  struct kernel_maternfivehalfs {
    /// @ingroup kernel_defaults
    BO_PARAM(double, sigma_sq, 1);
    /// @ingroup kernel_defaults
    BO_PARAM(double, l, 0.4);
  };
  struct kernel_exp {
    HMR_PARAM(double, sigma, 0.25);
  };
  struct hammer{
    HMR_PARAM(bool, stats_enabled, true);
    HMR_PARAM_STRING(name, "LL_hammer");
  };
  struct tolerance{
    HMR_PARAM(double, tol, 0.01);
  };

  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 10);
  };
      struct gp {
	HMR_PARAM(double, noise, 1e-6);
	HMR_PARAM(int, hp_period, 0);

      };


  struct distance : public defaults::distance {};
  struct opt_parallelrepeater : limbo::defaults::opt_parallelrepeater {};
  struct opt_rprop : public limbo::defaults::opt_rprop {};
}; 


struct Params2{
  struct kernel_exp {
    HMR_PARAM(double, sigma, 0.25);
  };
  struct hammer{
    HMR_PARAM(bool, stats_enabled, true);
    HMR_PARAM_STRING(name, "HL_hammer");
  };
  struct tolerance{
    HMR_PARAM(double, tol, 0.1);
  };
  struct stop_maxiterations {
    HMR_PARAM(int, iterations,1000);
  };
  struct gp : public defaults::gp{};
  struct distance {
    HMR_PARAM(double, offset, 0);
    HMR_PARAM(double, rate, 1);
    HMR_PARAM(double, diff, 0.1);
  };

  struct opt_parallelrepeater : limbo::defaults::opt_parallelrepeater {};
  struct opt_rprop : public limbo::defaults::opt_rprop {};
}; 



template<typename HMR, typename SYS>
class HMRwrapper{
public:
  typedef typename SYS::state_t state_t;
  
  HMRwrapper(HMR& hammer,SYS& sys):_hmr(hammer),_sys(sys){}

  state_t operator()(const typename HMR::state_t& target){
    _hmr.printStructure();
    std::cout<<"wrapper target "<< target.transpose() <<std::endl;
    _hmr.run(_sys, target, _sys.getCurrentState());
    std::cout<<"wrapper final "<< _sys.getCurrentState(0).transpose() <<std::endl;
    return _sys.getCurrentState(1);
  }

  state_t getCurrentState(){return _sys.getCurrentState(1);}

  HMR& _hmr;
  SYS& _sys;
};







class StateMachine{
public:
  StateMachine(){}

  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    //std::cout<<"here1"<<std::endl;
    Eigen::VectorXd action(2);
    Eigen::VectorXd error=target-current;
    if(error.tail(2).norm()>0.2){//far from the target
      double align=atan2(error(2),error(1))-current(0);
      if (align>M_PI)
	align-=2*M_PI;
      if (align<-M_PI)
	align+=2*M_PI;
      if(fabs(align)>0.1){
	action(0)=std::min(std::max(align/5,-0.2),0.2);
	action(1)=0;
      }
      else{
	  action(0)=0;
	  action(1)=std::min(error.tail(2).norm()/5,1.0);
      }
    }
    else{ //close to the target
      double ori_err=error(0);
      if (ori_err>M_PI)
	ori_err-=2*M_PI;
      if (ori_err<-M_PI)
	ori_err+=2*M_PI;
      action(0)=std::min(std::max(ori_err/5,-0.2),0.2);
      action(1)=std::min(error.tail(2).norm()/5,1.0)/10;
    }
    std::cout<<"error "<< error.transpose() <<" action: "<<action.transpose()<<std::endl;
    //std::cout<<"here2"<<std::endl;
    return action;

  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
  
private:
  
};


int main (int argc, char *argv[])
{
  std::srand(std::time(NULL));
  srand(time(NULL));
 
  typedef Eigen::VectorXd Action;
  typedef Eigen::VectorXd State;


  Vehicle car; 
  
  //-------- FIRST LAYER -----------//

  // Instatiation of the forward models
  typedef limbo::kernel::MaternFiveHalves<Params> Kernel_t;
  typedef limbo::mean::Data<Params> Mean_t;
  //forwardModels::GP<Params, Kernel_t, Mean_t, limbo::model::gp::KernelLFOpt<Params> > gp(4,2);
  forwardModels::GP<Params, Kernel_t, Mean_t> gp(4,2);
  forwardModels::LinearLeastSquare LLS; 

  // Instantiation of the Inverse models;
  Action v=Action::Zero(2);
  Action v1=v;
  v1(0)=1;
  v1(1)=0;
  inverseModels::MonoVal<Action> im_1(v1);
  Action v2=v;
  v2(0)=0;
  v2(1)=0;
  inverseModels::MonoVal<Action> im_2(v2);
  Action v3=v;
  v3(0)=-1;
  v3(1)=0;
  inverseModels::MonoVal<Action> im_3(v3);
  Action v4=v;
  v4(0)= 1;
  v4(1)= 1;
  inverseModels::MonoVal<Action> im_4(v4);
  Action v5=v;
  v5(0)=0;
  v5(1)=1;
  inverseModels::MonoVal<Action> im_5(v5);
  Action v6=v;
  v6(0)=-1;
  v6(1)=1;
  inverseModels::MonoVal<Action> im_6(v6);
  Action v7=v;
  v7(0)=1;
  v7(1)=-1;
  inverseModels::MonoVal<Action> im_7(v7);
  Action v8=v;
  v8(0)=0;
  v8(1)=-1;
  inverseModels::MonoVal<Action> im_8(v8);
  Action v9=v;
  v9(0)=-1;
  v9(1)=-1;
  inverseModels::MonoVal<Action> im_9(v9);

  // Instantiation of the architecture
  typedef boost::fusion::vector< stop::MaxIterations<Params>, stop::Tolerance<Params> > stop_t;
  typedef  Hammer<Params,scorefun<scoreupdator::NearestTarget<Params> >, stopcrit<stop_t> > hmr_t;
  hmr_t hammer;
  // Biding of the Inverse models
  hammer.bindInverseModel(im_1, im_2, im_3, im_4, im_5, im_6, im_7, im_8, im_9);
  // Biding of the forward models
  hammer.bindForwardModel(LLS);    
  hammer.bindForwardModel(gp);

  // Diplay the strcture of the architecture
  hammer.printStructure();


  Eigen::VectorXd t(2);
  t<<-0.02, -0.25;
  //  hammer.run( car, t, Eigen::VectorXd::Zero(2));

  
  //return 0;
  

  //-------- SECOND LAYER -----------//
  
  typedef boost::fusion::vector< stop::MaxIterations<Params2>, stop::Tolerance<Params2> > stop2_t;
  typedef  Hammer<Params2,selectfun<hammer::selector::NonDominated<Params> >, scorefun<scoreupdator::NearestTarget<Params> >, stopcrit<stop2_t> > hmr2_t;
  hmr2_t hammer2;
  forwardModels::GP<Params2, Kernel_t,limbo::mean::NullFunction<Params> > gp2(5,3);
  forwardModels::LinearLeastSquare LLS2; 


  Eigen::MatrixXd P=Eigen::MatrixXd(2,3);
  P<<1,0,0.2,0,1,0.2;
  P/=30;
  Eigen::MatrixXd I=Eigen::MatrixXd::Zero(2,3);
  Eigen::MatrixXd D=Eigen::MatrixXd::Zero(2,3);
  hammer::inverseModels::PID pid(P,I,D);

  
  StateMachine SM;
  FMOptimizer<forwardModels::GP<Params2, Kernel_t, limbo::mean::NullFunction<Params> > > fmopt(gp2,2);

  hammer2.bindInverseModel(pid);
  hammer2.bindInverseModel(SM);
  hammer2.bindInverseModel(fmopt);
  // Biding of the forward models
  hammer2.bindForwardModel(gp2);
  //hammer2.bindForwardModel(LLS2);
  

  hammer2.printStructure();
  

  typedef boost::fusion::vector<Vehicle, hmr_t, hmr2_t> compo_t;
  Hierarchy<3,compo_t> hierarchy;

  hierarchy.bindInverseModel<1>(im_1, im_2, im_3, im_4, im_5, im_6, im_7, im_8, im_9);
  hierarchy.bindForwardModel<1>(LLS,gp);
  
  hierarchy.bindInverseModel<2>(SM,pid,fmopt);
  hierarchy.bindForwardModel<2>(gp2);

  
  
  // Definition of the target
  Eigen::VectorXd target(3);
  target<<1,3,-2;
  std::cout<<"TARGET "<<target.transpose()<<std::endl;


  hierarchy.printStructure();
  hierarchy.run(target);
  return 0;
  //hammer2.run( car ,target, Eigen::VectorXd::Zero(3));
  //return 0;
  
  
  //hammer.learn(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);



  //HMRwrapper<hmr_t, Vehicle > hmrwrp(hammer, car) ;

  //Execution of the architecture
  //hammer2.run( hmrwrp,target, Eigen::VectorXd::Zero(3),true);


  return 0;
}
