#ifndef HAMMER_CONFIDUPDATOR_DISTANCE_HPP
#define HAMMER_CONFIDUPDATOR_DISTANCE_HPP

#include <hammer/tools/macros.hpp>


namespace hammer{

    
    namespace defaults {
      struct distance {
	HMR_PARAM(double, offset, 0);
	HMR_PARAM(double, rate, 1);

      };
    }
    
  namespace confidupdator{
    template<typename Params>
    struct Distance{
      double  operator()(const int& newVal,const int& prevVal,double confidence) const
      {      
	return confidence + (Params::distance::offset() - std::abs(newVal-prevVal))*Params::distance::rate();
      }
      double  operator()(const Eigen::VectorXd& newVal,const Eigen::VectorXd& prevVal,double confidence)const
      {
	return confidence + (Params::distance::offset() - (newVal-prevVal).norm())*Params::distance::rate();
      }
    };
    
    
  }
}

#endif
