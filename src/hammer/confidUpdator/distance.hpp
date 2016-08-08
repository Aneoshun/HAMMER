#ifndef HAMMER_CONFIDUPDATOR_DISTANCE_HPP
#define HAMMER_CONFIDUPDATOR_DISTANCE_HPP

#include <hammer/tools/macros.hpp>


namespace hammer{

    
    namespace defaults {
      struct distance {
	HMR_PARAM(double, offset, 0);
	HMR_PARAM(double, rate, 1);
	HMR_PARAM(double, diff, 1);

      };
    }
    
  namespace confidupdator{
    template<typename Params>
    struct Distance{
      double  operator()(double confidence, int newState, int predictedState, int suggested=0, int executed=0) const
      {      
	return confidence + (Params::distance::offset() - std::abs(newState-predictedState))*Params::distance::rate()*(1-std::min( std::fabs(executed-suggested), Params::distance::diff())/Params::distance::diff());
      }
      double  operator()(double confidence,const Eigen::VectorXd& newState, const Eigen::VectorXd& predictedState, const Eigen::VectorXd& suggested=Eigen::VectorXd::Zero(1), const Eigen::VectorXd& executed=Eigen::VectorXd::Zero(1)) const
      {
	return confidence +
	  (Params::distance::offset() - (newState-predictedState).norm()) *
	  Params::distance::rate() *
	  (1-std::min( (executed-suggested).norm() , Params::distance::diff())/Params::distance::diff());
      }
    };
    
    
  }
}

#endif
