#ifndef HAMMER_STOP_CHAIN_CRITERIA_HPP
#define HAMMER_STOP_CHAIN_CRITERIA_HPP

namespace hammer {
    namespace stop {
        template <typename HMR>
        struct ChainCriteria {
            typedef bool result_type;
	  ChainCriteria(const HMR& hmr,const typename HMR::state_t& target) : _hmr(hmr), _target(target){}

            template <typename stopping_criterion>
            bool operator()(bool state, stopping_criterion stop) const
            {
	      return state || stop(_hmr, _target);
            }

        protected:
	  const HMR& _hmr;
	  const typename HMR::state_t& _target;
	};
    }
}

#endif
