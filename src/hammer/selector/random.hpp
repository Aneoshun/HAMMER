#ifndef HAMMER_SELECTOR_RANDOM_HPP
#define HAMMER_SELECTOR_RANDOM_HPP



namespace hammer{
  namespace selector{
    template<typename Params>
    struct Random{
      template<typename ModelPair>
      typename std::shared_ptr<ModelPair> operator()(const std::vector< std::shared_ptr<ModelPair> >& pairs)const
      {
	tools::rgen_int_t rgen(0, pairs.size()-1);
	int ind = rgen.rand();
	return pairs[ind];
      }
    };

  }
}

#endif
