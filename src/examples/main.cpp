// A simple program that computes the square root of a number
//#define  EIGEN_DONT_PARALLELIZE
#include <iostream>
#include <algorithm>    // std::max

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define EIGEN_STACK_ALLOCATION_LIMIT 3000000

#include "limbo/limbo.hpp"

#include "./hammer/hammerConfig.h"
#include "./hammer/hammer.hpp"
#include "./hammer/forwardModels/simple.hpp"
#include "./hammer/selector/nearestTarget.hpp"
#include "./hammer/confidUpdator/distance.hpp"
#include "./hammer/inverseModels/simple.hpp"


using namespace hammer;

struct Params{
  struct kernel_exp {
    HMR_PARAM(double, sigma, 0.25);
  };
  struct hammer{
    HMR_DYN_PARAM(Eigen::VectorXd, target);
    HMR_PARAM(bool, stats_enabled, true);
  };

  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 500);
  };
  
  struct distance : public defaults::distance {};
}; 

HMR_DECLARE_DYN_PARAM(Eigen::VectorXd,Params::hammer,target);


struct AllGood{
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
    Eigen::VectorXd a=Eigen::VectorXd::Ones(1);

    Eigen::VectorXd diff=current-target;
    //std::cout<<target <<" diff : "<<diff<<std::endl;
    //System.out.println(diff);                                                                                                                                         
    if(diff(0)>0.2){
      a(0)=2;
    }
    else if(diff(0)< -0.2 && diff(0)>-2){
      a(0)=3;
    }
    else if(diff(0)< -2){
      a(0)=4;
    }
    //std::cout<<"action : "<< a<<std::endl;
    return a;
  }

  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

};



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
  State operator()( ) { //for imitation learning
    //GlucoseLv_mmol=randomGenerator.nextDouble()*8+4;//4-8-12
    //    std::cout<< "received: "<<action<<std::endl;
    /*Action action=AllGood().predict(glucose,Params::hammer::target() );
    if(std::abs(action[0] -4)<0.01)
      glucose(0)+=0.2;
    else if(std::abs(action[0]-3)<0.01) 
      glucose[0]+=0.1;
    else if(std::abs(action[0]-2)<0.01)
    glucose[0]-=0.1;*/
    //else assert(0);
    glucose+=Eigen::VectorXd::Random(1)/3;
    std::cout<< glucose<<std::endl;

    return glucose;
  }

};


struct GP{
  GP():_gp(5,1){}
  Eigen::VectorXd predict (const Eigen::VectorXd& action, const Eigen::VectorXd& current) const{

    //std::cout<<"nb : "<<_gp.nb_samples()<<std::endl;
    
    //Eigen::Matrix<float,500,500> a = Eigen::Matrix<float,500,500>::Random();
    
    //Eigen::Matrix<float,500,500> b(a*a); 

    Eigen::VectorXd x=Eigen::VectorXd::Zero(5);
    x(0)=current(0)/15;
    
    //int index=round(action(0))+1;
    //std::cout<<"index "<<index<<std::endl;
    x(round(action(0)))=1;
    
    return _gp.mu(x)*15;
    
  }

  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){
    //WARNING, NOT REENTRANT
    Eigen::VectorXd sample=Eigen::VectorXd::Zero(5);
    Eigen::VectorXd ob(1);
    ob(0)=next(0)/15;
    sample(0)=prev(0)/15;
    sample(round(action(0)))=1;
    
    obs.push_back(ob);
    samples.push_back(sample);
    _gp.compute(samples,obs,0.01);

  }
    std::vector<Eigen::VectorXd> obs;
    std::vector<Eigen::VectorXd> samples;


  void init()
  {
    Eigen::VectorXd sample=Eigen::VectorXd::Zero(5);
    Eigen::VectorXd ob(1);
    
    for (size_t i=0; i<100; i++)
      {
	sample=Eigen::VectorXd::Zero(5);
	int action=1+rand()%4;
	Eigen::VectorXd initstate=(Eigen::VectorXd::Random(1).array()+1)/2;
	sample(0)=initstate(0);
	sample(action)=1;
	ob=initstate*15;
	switch(action)
	  {
	  case 2:
	    ob(0) = ob(0)-0.1;
	    break;
	  case 3:
	    ob(0) = ob(0)+0.1;
	    break;
	  case 4:
	    ob(0) = ob(0)+0.2;
	    break;
	  }
                
	ob/=15;
	obs.push_back(ob);
	samples.push_back(sample);
      }
    _gp.compute(samples,obs,0.01);
    
  }
limbo :: model::GP<Params, limbo::kernel::Exp<Params>, limbo::mean::Data<Params> > _gp;

  

};






int main (int argc, char *argv[])
{
  std::srand(std::time(NULL));
  srand(time(NULL));
  
  std::cout<<rand()<<std::endl;
std::cout<<rand()<<std::endl;
std::cout<<rand()<<std::endl;

  
  //if(argc>1)
  //std::cout<<"Version "<< Hammer_VERSION_MAJOR<<"."<< Hammer_VERSION_MINOR<<std::endl;

  // Instatiation of the forward models
  forwardModels::Simple<> fm;
  GP gp;
  //  gp.init();

  // Instantiation of the Inverse models;
  inverseModels::MonoVal<1> im_1;
  inverseModels::MonoVal<2> im_2;
  inverseModels::MonoVal<3> im_3;
  inverseModels::MonoVal<4> im_4;

  AllGood im_5;
  // Instantiation of the architecture
  Hammer<Params, selectfun<selector::NearestTarget<Params> > > hammer;

 





 // Biding of the Inverse models
  //hammer.bindInverseModel(im_5);
  hammer.bindInverseModel(im_1);
  hammer.bindInverseModel(im_2);
  hammer.bindInverseModel(im_3);
  hammer.bindInverseModel(im_4);

  // Biding of the forward models
  for(int i=0;i<atoi(argv[1]);i++)
    hammer.bindForwardModel(gp);    

  hammer.printStructure();


  // Execution of the architecture
  DummyGlucose<> exp;
  Eigen::VectorXd target(1);
  target(0)=8;
  Params::hammer::set_target(target);
  
  std::cout<<"TARGET "<<Params::hammer::target()<<std::endl;    
  //hammer.run(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
  hammer.learn(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
  
  return 0;
}
