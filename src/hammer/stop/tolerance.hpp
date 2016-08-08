#ifndef HAMMER_STOP_TOLERANCE_HPP
#define HAMMER_STOP_TOLERANCE_HPP

#include <hammer/tools/macros.hpp>

namespace hammer {
    namespace defaults {
        struct tolerance {
            HMR_PARAM(double, tol, 0.1);
        };
    }
    namespace stop {
        template <typename Params>
        struct Tolerance {
            Tolerance() {}

            template <typename HMR>
            bool operator()(const HMR& hmr,const typename HMR::state_t& target)
            {
	      
	      return (hmr.getCurrentState()-target).cwiseProduct(Params::nearesttarget::proj()).norm() <= Params::tolerance::tol();
            }
        };
    }
}

#endif
