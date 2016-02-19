#ifndef HAMMER_STAT_PREDICTIONS_HPP
#define HAMMER_STAT_PREDICTIONS_HPP

#include <hammer/stat/stat_base.hpp>

namespace hammer {
  namespace stat {
    template <typename Params>
    struct Predictions : public StatBase<Params> {
      template <typename HMR>
      void operator()(const HMR& hmr)
      {
	if (!hmr.stats_enabled())
	  return;

	this->_create_log_file(hmr, "predictions.dat");

	if (hmr.current_iteration() == 0) {
	  (*this->_log_file) << "#iteration observation" << std::endl;
	}
	
	(*this->_log_file) << hmr.current_iteration() << ": "<< std::endl;
	std::for_each(hmr.getPairInterfaces().begin(),hmr.getPairInterfaces().end(), [=](const std::shared_ptr<typename HMR::ModelPair_t>& fm ) {(*this->_log_file) <<"      Conf: "<<(*fm).getConfidence()<< "  Pred state: "<<(*fm).getState()<<"   Suggest action: "<<(*fm).getAction()<<std::endl;} );
      }
    };
  }
}

#endif
