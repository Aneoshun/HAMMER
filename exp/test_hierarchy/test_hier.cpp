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


typedef Eigen::VectorXd Action;
typedef Eigen::VectorXd State;


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
    BO_PARAM(double, l, 1);
  };
  struct kernel_exp {
    HMR_PARAM(double, sigma, 0.25);
  };
  struct hammer{
    HMR_PARAM(bool, stats_enabled, true);
    HMR_PARAM_STRING(name, "LL_hammer");
  };

  struct nearesttarget{
    HMR_PARAM_VECTOR(double, proj, 1,1 );
  };

  struct tolerance{
    HMR_PARAM(double, tol, 0.1);
  };

  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 1);
  };
  struct gp {
    HMR_PARAM(double, noise, 1e-6);
    HMR_PARAM(int, hp_period, 0);
  };
  struct ucb {
    HMR_PARAM(double, alpha, 0.2);
  };
  

  struct distance : public defaults::distance {};
  struct opt_parallelrepeater : limbo::defaults::opt_parallelrepeater {};
  struct opt_rprop : public limbo::defaults::opt_rprop {};
}; 


struct ParamsNT{
  struct tolerance{
    HMR_PARAM(double, tol, 0.05);
  };

  struct nearesttarget{
    HMR_PARAM_VECTOR(double, proj, 0,0,0,1,1 );
  };

    struct stop_maxiterations {
    HMR_PARAM(int, iterations, 1000);
  };

};


template<typename Action>
class MonoDelta{
public:
  MonoDelta(Action act):_act(act){}
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    return current.head(2)+_act;
  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
  Action _act;
};



class StateMachine{
public:
  StateMachine(){}

  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    //std::cout<<"here1"<<std::endl;
    Eigen::VectorXd action(2);
    Eigen::VectorXd error=target-current;
    double align=atan2(error(3),error(2))-atan2( current(1), current(0));

    if (align>M_PI)
      align-=2*M_PI;
    if (align<-M_PI)
      align+=2*M_PI;

    
    if(error.tail(2).norm()>0.2){//far from the target

      if(fabs(align)>0.1){
	action(0)=std::min(std::max(align/10,-0.2),0.2);
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
    std::cout<<"align "<<align<<" error "<< error.transpose() <<" action: "<<action.transpose()<<std::endl;
    //std::cout<<"here2"<<std::endl;
    return action;

  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
  
private:
  
};


template<typename IM>
std::vector<IM> genDiscreteActions(double scale)
{
  std::vector<IM > res;
  Action v=Action::Zero(2);
  res.push_back(IM(v*scale));

  v(0)=0;
  v(1)=1;
  res.push_back(IM(v*scale));
  v(0)=0;
  v(1)=-1;
  res.push_back(IM(v*scale));


  v(0)=1;
  v(1)=0;
  res.push_back(IM(v*scale));
  v(0)=1;
  v(1)=1;
  res.push_back(IM(v*scale));
  v(0)=1;
  v(1)=-1;
  res.push_back(IM(v*scale));


  v(0)=-1;
  v(1)=0;
  res.push_back(IM(v*scale));
  v(0)=-1;
  v(1)=1;
  res.push_back(IM(v*scale));
  v(0)=-1;
  v(1)=-1;
  res.push_back(IM(v*scale));
  return res;
}


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


int main (int argc, char *argv[])
{
  std::srand(std::time(NULL));
  srand(time(NULL));
  
  // Instantiation of the architecture
  typedef boost::fusion::vector< stop::MaxIterations<Params>, stop::Tolerance<Params> > stop_t;
  typedef Hammer<Params,scorefun<scoreupdator::NearestTarget<Params> >, selectfun<selector::NonDominated<Params> >, stopcrit<stop_t> > hmr1_t;
  typedef boost::fusion::vector< stop::MaxIterations<ParamsNT>, stop::Tolerance<ParamsNT> > stop2_t;
  typedef Hammer<Params,scorefun<scoreupdator::NearestTarget<ParamsNT> >, selectfun<selector::NonDominated<Params> >, stopcrit<stop2_t> > hmr2_t;
  typedef boost::fusion::vector< stop::MaxIterations<Params>, stop::Tolerance<Params> > stop3_t;
  typedef Hammer<Params,scorefun<scoreupdator::NearestTarget<Params> >, stopcrit<stop3_t> > hmr3_t;

  typedef boost::fusion::vector<Vehicle, hmr1_t, hmr2_t, hmr3_t> compo_t;
  Hierarchy<3,compo_t> hierarchy;

  std::vector<inverseModels::MonoVal<Eigen::VectorXd> > IMlvl1=genDiscreteActions<inverseModels::MonoVal<Eigen::VectorXd> >(1);  
  for(auto& im:IMlvl1)
    hierarchy.bindInverseModel<1>(im);  
  forwardModels::LinearLeastSquare<> LLS1;
  hierarchy.bindForwardModel<1>(LLS1);

  std::vector<MonoDelta<Eigen::VectorXd> > IMlvl2=genDiscreteActions<MonoDelta<Eigen::VectorXd> >(0.2);
  for(auto& im:IMlvl2)
    hierarchy.bindInverseModel<2>(im);  
  forwardModels::LinearLeastSquare<Extension> LLS2;
  hierarchy.bindForwardModel<2>(LLS2);
  
  //typedef limbo::kernel::MaternFiveHalves<Params> Kernel_t;
  typedef limbo::kernel::SquaredExpARD<Params> Kernel_t;
  typedef limbo::mean::Data<Params> Mean_t;
  forwardModels::GP<Params, Kernel_t, Mean_t, limbo::model::gp::KernelLFOpt<Params> > gp2(7,5);
  //forwardModels::GP<Params, Kernel_t,limbo::mean::NullFunction<Params> > gp2(7,5);
  //hierarchy.bindForwardModel<2>(gp2);
  //inverseModels::MonoVal<Action>(target);
  //forwardModels::LinearLeastSquare LLS2;
  //hierarchy.bindForwardModel<2>(LLS2);


  State target(5);
  target<<0,0,0,0,0.5;
  
  
  hierarchy.printStructure();
  hierarchy.run(target);

  
  return 0;
}
