// A simple program that computes the square root of a number
//#define  EIGEN_DONT_PARALLELIZE
#include <iostream>
#include <algorithm>    // std::max

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define EIGEN_STACK_ALLOCATION_LIMIT 3000000



#include "./hammer/hammerConfig.h"
#include "./hammer/hammer.hpp"
#include "./hammer/forwardModels/simple.hpp"
#include "./hammer/forwardModels/gp.hpp"
#include "./hammer/selector/nearestTarget.hpp"
#include "./hammer/confidUpdator/distance.hpp"
#include "./hammer/inverseModels/simple.hpp"


using namespace hammer;

// Definition of the parameters
struct Params{
  struct kernel_exp {
    HMR_PARAM(double, sigma, 0.25);
  };
  struct hammer{
    HMR_DYN_PARAM(Eigen::VectorXd, target);
    HMR_PARAM(bool, stats_enabled, true);
  };

  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 100);
  };
  struct gp : public defaults::gp{};
  struct distance : public defaults::distance {};
}; 

HMR_DECLARE_DYN_PARAM(Eigen::VectorXd,Params::hammer,target);



// Example of an Inverse Model
struct PerfectFM{
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    Eigen::VectorXd a=Eigen::VectorXd::Ones(1);

    Eigen::VectorXd diff=current-target;
    if(diff(0)>0.2){
      a(0)=2;
    }
    else if(diff(0)< -0.2 && diff(0)>-2){
      a(0)=3;
    }
    else if(diff(0)< -2){
      a(0)=4;
    }
    return a;
  }

  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

};


// Definition of the problem/scenario
template<typename State=Eigen::VectorXd, typename Action=Eigen::VectorXd>
struct DummyGlucose{
  State glucose;
  DummyGlucose(){
    glucose=Eigen::VectorXd(1);
    glucose(0)= ((float)rand()/RAND_MAX)*8+4;
    std::cout<< glucose<<std::endl;
}
  State operator()(const Action& action ) {
    //GlucoseLv_mmol=randomGenerator.nextDouble()*8+4;//4-8-12
    std::cout<< "received: "<<action<<std::endl;

    if(std::abs(action[0] -4)<0.01)
      glucose(0)+=0.2;
    else if(std::abs(action[0]-3)<0.01) 
      glucose[0]+=0.1;
    else if(std::abs(action[0]-2)<0.01)
      glucose[0]-=0.1;
    //else assert(0);
    glucose+=Eigen::VectorXd::Random(1)/3;
    std::cout<< glucose<<std::endl;

    return glucose;
  }
  State operator()( ) {   //for imitation learning
    return this->operator()(PerfectFM().predict(glucose,Params::hammer::target() ));
  }

};



int main (int argc, char *argv[])
{
  std::srand(std::time(NULL));
  srand(time(NULL));
  
  
  // Instatiation of the forward models
  forwardModels::GP<Params> gp;
  //  gp.init();

  // Instantiation of the Inverse models;
  inverseModels::MonoVal<1> im_1;
  inverseModels::MonoVal<2> im_2;
  inverseModels::MonoVal<3> im_3;
  inverseModels::MonoVal<4> im_4;

  // Instantiation of the architecture
  Hammer<Params, selectfun<selector::NearestTarget<Params> > > hammer;

 
  // Biding of the Inverse models
  //hammer.bindInverseModel(im_5);
  hammer.bindInverseModel(im_1);
  hammer.bindInverseModel(im_2);
  hammer.bindInverseModel(im_3);
  hammer.bindInverseModel(im_4);

  // Biding of the forward models
  hammer.bindForwardModel(gp);    
  
  // Diplay the strcture of the architecture
  hammer.printStructure();


  // Instanciation of the scenario
  DummyGlucose<> exp;
  
  // Definition of the target
  Eigen::VectorXd target(1);
  target(0)=8;
  Params::hammer::set_target(target);
  

  //Execution of the architecture
  std::cout<<"TARGET "<<Params::hammer::target()<<std::endl;    

  hammer.learn(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
  hammer.run(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
  
  return 0;
}
