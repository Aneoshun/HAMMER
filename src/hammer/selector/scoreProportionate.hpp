#ifndef HAMMER_SELECTOR_SCOREPROPORTIONATE_HPP
#define HAMMER_SELECTOR_SCOREPROPORTIONATE_HPP

#include "tbb/parallel_reduce.h"
#include "tbb/parallel_sort.h"
#include "hammer/tools.hpp"

namespace hammer{
  namespace selector{
    template<typename Params>
    struct ScoreProportionate{
      template<typename ModelPair>
      typename std::shared_ptr<ModelPair> operator()(const std::vector< std::shared_ptr<ModelPair> >& pairs)const
      {
	typedef typename std::vector< std::shared_ptr<ModelPair> >::iterator it_t;
	//copy of the vector that will be sorted
	std::vector< std::shared_ptr<ModelPair> > v(pairs);
	tbb::parallel_sort(v.begin(),v.end(),Comparator());
	double max=(*v.begin())->getScore();
	double min=(*(--v.end()))->getScore();
	if(min!=max){
	  tools::rgen_double_t rgen(0.0, 1.0);
	  double r = rgen.rand(); 
	  //double sum=0;
	  //for(it_t it=v.begin(); it!=v.end();++it)
	  //  sum+=((*it)->getScore()-min)/(max-min);
	  
	  typedef tbb::blocked_range<it_t> range_type;
	  double sum= tbb::parallel_reduce(
					   range_type( v.begin(), v.end() ), 0.0,
					   [](const range_type& r, double value)->double {
					     return std::accumulate(r.begin(),r.end(),value,[]( double const & current, std::shared_ptr<ModelPair> const& p)
								    { return current + p->getScore(); });
					   },
					   std::plus<double>()
					   );
	  //
	  //auto sum=tbb::parallel_reduce(range_type(v.begin(),v.end()), 0,
	  //			       [](range_type const&r, double init)
	  //				{ return std::accumulate(r.begin(),r.end(),init,[](std::shared_ptr<ModelPair> const& p, double current)
	  // { return current+=p->getScore(); }); },
	  //				[](std::shared_ptr<ModelPair> const& p, double current)
	  //				{ return current+=p->getScore(); });


	  it_t it=v.begin();
	  double p = ((*it)->getScore()-min)/(sum-v.size()*min);
	  while (p<r)
	    {
	      it++;
	      p+=((*it)->getScore()-min)/(sum-v.size()*min);
	    }
	  return *it;
	}else{ //they all have the same score
	  tools::rgen_int_t rgen(0, v.size()-1);
	  return v[rgen.rand()];
	}
	return v[0];//should not be reached
	
      }
      struct Comparator{
	template<typename T1, typename T2>
	bool operator()(const T1& t1, const T2& t2)const
	{
	  return (t1)->getScore() > (t2)->getScore(); //descending order
	}
      };
    };

  }
}

#endif
