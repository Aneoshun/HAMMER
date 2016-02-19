#ifndef HAMMER_STOP_MAX_ITERATIONS_HPP
#define HAMMER_STOP_MAX_ITERATIONS_HPP

#include <hammer/tools/macros.hpp>

namespace hammer {
    namespace defaults {
        struct stop_maxiterations {
            HMR_PARAM(int, iterations, 190);
        };
    }
    namespace stop {
        template <typename Params>
        struct MaxIterations {
            MaxIterations() {}

            template <typename HMR>
            bool operator()(const HMR& hmr)
            {
                return hmr.current_iteration() >= Params::stop_maxiterations::iterations();
            }
        };
    }
}

#endif
