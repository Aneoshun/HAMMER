#ifndef HAMMER_SELECTOR_PSEUDOMAX_HPP
#define HAMMER_SELECTOR_PSEUDOMAX_HPP



namespace hammer{
  namespace selector{

    namespace defaults{
      struct pseudomax{
	HMR_PARAM(double, prob, 0.05);
      };
    }
    
    template<typename Params>
    struct PseudoMax{
      template<typename ModelPair>
      typename std::shared_ptr<ModelPair> operator()(const std::vector< std::shared_ptr<ModelPair> >& pairs)const
      {

	tools::rgen_double_t rgen(0.0, 1);
	double r = rgen.rand();
	if(r<Params::pseudomax::prob()){
	  tools::rgen_int_t rgen(0, pairs.size()-1);
	  int ind = rgen.rand();
	  return pairs[ind];
	}
	else
	  return *std::max_element(pairs.begin(),pairs.end(), Comparator()); 
      }
      struct Comparator{
	template<typename T1, typename T2>
	bool operator()(const T1& t1, const T2& t2)const
	{
	  return (t1)->getScore() < (t2)->getScore();
	}
      };
    };

  }
}

#endif
