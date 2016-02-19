#ifndef HAMMER_SELECTOR_CONFIDENCE_HPP
#define HAMMER_SELECTOR_CONFIDENCE_HPP



namespace hammer{
  namespace selector{
    template<typename Params>
    struct Confidence{
      template<typename T1, typename T2>
      bool operator()(const T1& t1, const T2& t2)const 
      {
	return (t1)->getConfidence() < (t2)->getConfidence();
      }
    };

  }
}

#endif
