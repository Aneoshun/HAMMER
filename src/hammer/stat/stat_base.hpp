#ifndef HAMMER_STAT_STAT_BASE_HPP
#define HAMMER_STAT_STAT_BASE_HPP

#include <fstream>
#include <string>

#include <memory>

namespace hammer {
    namespace stat {
        template <typename Params>
        struct StatBase {
            StatBase() {}

            template <typename HMR>
            void operator()(const HMR& hmr)
            {
                assert(false);
            }

        protected:
            std::shared_ptr<std::ofstream> _log_file;

	  template<typename Data>
	  const Data& convert(const Data& data){return data;} //does nothing


	  const Eigen::MatrixXd convert(const Eigen::VectorXd& data){//tranposes the data
	    if(data.rows()>1)
	      return data.transpose();
	    else
	      return data;
	  } 

            template <typename HMR>
            void _create_log_file( const HMR& hmr, const std::string& name)
            {
                if (!_log_file && hmr.stats_enabled()) {
                    std::string log = hmr.res_dir() + "/" + name;
                    _log_file = std::make_shared<std::ofstream>(log.c_str());
                }
            }
        };
    }
}

#endif
