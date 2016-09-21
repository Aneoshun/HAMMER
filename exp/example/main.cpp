#include <iostream>
#include <algorithm>    // std::max


#define EIGEN_STACK_ALLOCATION_LIMIT 3000000




#include "./hammer/hammer.hpp"


using namespace hammer;

// Definition of the parameters
struct Params{
struct kernel_squared_exp_ard {
    HMR_PARAM(int, k, 0); //equivalent to the standard exp ARD
    HMR_PARAM(double, sigma_sq, 1);
  };
  struct nearesttarget{
    HMR_PARAM_VECTOR(double, proj, 1 );
  };
  
  struct kernel_exp {
    HMR_PARAM(double, sigma, 0.25);
  };
  struct hammer{
    HMR_DYN_PARAM(Eigen::VectorXd, target);
    HMR_PARAM_STRING(name, "Hammer")
    HMR_PARAM(bool, stats_enabled, true);
  };

  struct stop_maxiterations {
    HMR_PARAM(int, iterations, 100);
  };
  struct gp : public defaults::gp{};
  struct distance : public defaults::distance {};
  struct opt_parallelrepeater : limbo::defaults::opt_parallelrepeater {};
  struct opt_rprop : public limbo::defaults::opt_rprop {};
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
    std::cout<< "received: "<<action.transpose()<<std::endl;
    Action t(4);
    t<<1,2,3,4;
    int a=(t.transpose()*action).norm();
    switch(a){
    case 4:
      glucose[0]+=0.2;
      break;
    case 3:
    glucose[0]+=0.1;
    break;
    case 2:
      glucose[0]-=0.1;
      break;
    case 1:
      break;
    default:
      assert(0);
    }
    //glucose+=Eigen::VectorXd::Random(1)/3;
    std::cout<<"new glucose" << glucose<<std::endl;
    
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


  // Instanciation of the scenario
  DummyGlucose<> exp;

  
  typedef Eigen::VectorXd Action;
  typedef Eigen::VectorXd State;


  // Instatiation of the forward models
  typedef limbo::kernel::SquaredExpARD<Params> Kernel_t;
  typedef limbo::mean::Data<Params> Mean_t;
  forwardModels::GP<Params, Kernel_t, Mean_t, limbo::model::gp::KernelLFOpt<Params> > gp;
  //forwardModels::GP<Params > gp;


  /*std::vector<Eigen::VectorXd> obs;
  std::vector<Eigen::VectorXd> samples;
    for(size_t i=0;i<100;i++){
    Action a=Eigen::VectorXd(1);
    a(0)=rand()%4+1;
    State s=exp.glucose;
    State ns=exp(a);
    obs.push_back(ns);
    Eigen::VectorXd sam(2);
    sam<<s,a;
    samples.push_back(sam);
    }
  gp.init(obs,samples);
  */
  // Instantiation of the Inverse models;
  Action v=Action::Zero(4);
  Action v1=v;
  v1(0)=1;
  inverseModels::MonoVal<Action> im_1(v1);
  Action v2=v;
  v2(1)=1;
  inverseModels::MonoVal<Action> im_2(v2);
  Action v3=v;
  v3(2)=1;
  inverseModels::MonoVal<Action> im_3(v3);
  Action v4=v;
  v4(3)=1;
  inverseModels::MonoVal<Action> im_4(v4);

  // Instantiation of the architecture
  Hammer<Params,selectfun<hammer::selector::ScoreProportionate<Params> >,scorefun<scoreupdator::NearestTarget<Params> > > hammer;

 
  // Biding of the Inverse models
  hammer.bindInverseModel(im_1);
  hammer.bindInverseModel(im_2);
  hammer.bindInverseModel(im_3);
  hammer.bindInverseModel(im_4);

  // Biding of the forward models
  hammer.bindForwardModel(gp);    
  
  // Diplay the strcture of the architecture
  hammer.printStructure();

  
  // Definition of the target
  Eigen::VectorXd target(1);
  target(0)=8;
  Params::hammer::set_target(target);
  

  //Execution of the architecture
  std::cout<<"TARGET "<<Params::hammer::target()<<std::endl;    
  
  //hammer.learn(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
  hammer.run(exp, Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
  /*hammer.print_prediction();
  for(int i=0;i<100;i++)
    {
      Action a=hammer.suggestAction(Params::hammer::target(), Eigen::VectorXd::Random(1)*15);
      hammer.print_prediction();
      hammer.updateModels(exp(a));
      }*/

  return 0;
}
