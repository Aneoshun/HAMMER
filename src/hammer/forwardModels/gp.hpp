#ifndef GP_FM_HPP
#define GP_FM_HPP

// Simple wrapper around the GP implementation in LIMBO



#include "limbo/limbo.hpp"
namespace hammer {
  
    namespace defaults {
      struct gp {
	HMR_PARAM(double, noise, 1e-6);
      };
    }
  namespace forwardModels{
    template<typename Params, typename kernelFun=limbo::kernel::Exp<Params>, typename meanFun=limbo::mean::Data<Params>>
    class GP{
    public:
      GP(size_t dimIn=2, size_t dimOut=1):_gp(dimIn,dimOut){} //dim in =2 for concatenation between state + action

      Eigen::VectorXd predict (const Eigen::VectorXd& action, const Eigen::VectorXd& current) const{
	Eigen::VectorXd x(current.rows() + action.rows());
	x<<current,action;
	return _gp.mu(x);
	
      }
      
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){
	Eigen::VectorXd sample(prev.rows()+action.rows());
	sample<<prev,action;
	std::lock_guard<std::mutex> lock(_mutex);
	_observations.push_back(next);
	_samples.push_back(sample);
	_gp.compute(_samples,_observations,Params::gp::noise());
      }

      void init(const std::vector<Eigen::VectorXd>& observations, const std::vector<Eigen::VectorXd>& samples)
      {
	_samples=samples;
	_observations=observations;
	_gp.compute(_samples,_observations,Params::gp());
	
      }
    private:
      std::mutex _mutex; 
      limbo :: model::GP<Params, kernelFun, meanFun > _gp;
      
      std::vector<Eigen::VectorXd> _observations;
      std::vector<Eigen::VectorXd> _samples;
      
      
    };
  }
}    

#endif
