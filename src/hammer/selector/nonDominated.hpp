#ifndef HAMMER_SELECTOR_NONDOMINATED_HPP
#define HAMMER_SELECTOR_NONDOMINATED_HPP


namespace hammer{
  namespace selector{
    template<typename Params>
    struct NonDominated{
      template<typename ModelPair>
      typename std::shared_ptr<ModelPair> operator()(const std::vector< std::shared_ptr<ModelPair> >& pairs)const
      {
	std::vector< std::shared_ptr<ModelPair> > nonDominated;
	for(auto const& current: pairs) {
	  if(!isDominated(current, pairs))
	    nonDominated.push_back(current);
	}

	assert(nonDominated.size());
	if(nonDominated.size()==1)
	  return nonDominated[0];
	else
	  {
	    tools::rgen_int_t rgen(0, nonDominated.size()-1);
	    int ind = rgen.rand();
	    return nonDominated[ind];
	  }
	
      }
      template<typename ModelPair>
      bool isDominated(const std::shared_ptr<ModelPair>& current, const std::vector< std::shared_ptr<ModelPair> >& pairs)const
      {
	for(auto const& p: pairs) {
	  if (dominates(p, current))
	    return true;
	}
	return false;
      }

      template<typename ModelPair>
      bool dominates(const std::shared_ptr<ModelPair>& p1,const std::shared_ptr<ModelPair>& p2) const
      {
	bool c1=p1->getConfidence()>=p2->getConfidence() && p1->getScore()>=p2->getScore();
	bool c2=p1->getConfidence()>p2->getConfidence() || p1->getScore()>p2->getScore();
	return c1 && c2;
      }
    };

  }
}

#endif
