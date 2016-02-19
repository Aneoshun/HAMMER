#ifndef HAMMER_STAT_SELECTEDACTIONS_HPP
#define HAMMER_STAT_SELECTEDACTIONS_HPP

#include <hammer/stat/stat_base.hpp>

namespace hammer {
  namespace stat {
    template <typename Params>
    struct SelectedActions : public StatBase<Params> {
      template <typename HMR>
      void operator()(const HMR& hmr)
      {
	if (!hmr.stats_enabled())
	  return;

	this->_create_log_file(hmr, "selectedActions.dat");

	if (hmr.current_iteration() == 0) {
	  (*this->_log_file) << "#iteration observation" << std::endl;
	}
	
	(*this->_log_file) << hmr.current_iteration() << ": "<< hmr.getSelectedAction()<<std::endl;
      }
    };
  }
}

#endif
