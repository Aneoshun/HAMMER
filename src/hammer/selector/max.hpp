#ifndef HAMMER_SELECTOR_MAX_HPP
#define HAMMER_SELECTOR_MAX_HPP



namespace hammer{
  namespace selector{
    template<typename Params>
    struct Max{
      template<typename ModelPair>
      typename std::shared_ptr<ModelPair> operator()(const std::vector< std::shared_ptr<ModelPair> >& pairs)const
      {
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
