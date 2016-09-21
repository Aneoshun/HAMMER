#ifndef GP_FM_HPP
#define GP_FM_HPP

// Simple wrapper around the GP implementation in LIMBO



#include "limbo/limbo.hpp"
namespace hammer {
  
    namespace defaults {
      struct gp {
	HMR_PARAM(double, noise, 1e-6);
	HMR_PARAM(int, hp_period, 0);
	HMR_PARAM(bool, transition, true);
      };
    }
  namespace forwardModels{
    template<typename Params, typename kernelFun=limbo::kernel::Exp<Params>, typename meanFun=limbo::mean::Data<Params>,typename hparamOpt = limbo::model::gp::NoLFOpt<Params>>
    class GP{
    public:
      GP(int dim_in=1, int dim_out=1):_gp(dim_in,dim_out){}

      Eigen::VectorXd predict (const Eigen::VectorXd& action, const Eigen::VectorXd& current) const{
	
	Eigen::VectorXd x(current.rows() + action.rows());
	x<<current,action;
	if( Params::gp::transition())
	  return _gp.mu(x)+current;
	else
	  return _gp.mu(x);
	
      }

      double uncertainty (const Eigen::VectorXd& action, const Eigen::VectorXd& current) const{
	Eigen::VectorXd x(current.rows() + action.rows());
	x<<current,action;
	return _gp.sigma(x);
	
      }

      
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){
       	Eigen::VectorXd sample(prev.rows()+action.rows());
	sample<<prev,action;
	std::lock_guard<std::mutex> lock(_mutex);
	if( Params::gp::transition() )
	  _observations.push_back(next-prev);
	else
	  _observations.push_back(next);
	_samples.push_back(sample);
	_gp.compute(_samples,_observations,Eigen::VectorXd::Constant(this->_observations.size(), Params::gp::noise()));
	if (Params::gp::hp_period() > 0 && (_samples.size() + 1) % Params::gp::hp_period() == 0)
	  _gp.optimize_hyperparams();
	
      }

      void init(const std::vector<Eigen::VectorXd>& observations, const std::vector<Eigen::VectorXd>& samples)
      {
	std::cout<<"Warning, the samples should contains CurrentState+Action and the observations should be like NextState-CurrentState"<<std::endl;
	_samples=samples;
	_observations=observations;
	_gp.compute(_samples,_observations,Params::gp::noise());
	
      }
    private:
      std::mutex _mutex; 
      limbo :: model::GP<Params, kernelFun, meanFun,hparamOpt > _gp;
      
      std::vector<Eigen::VectorXd> _observations;
      std::vector<Eigen::VectorXd> _samples;
      
      
    };
  }
}    

#endif
