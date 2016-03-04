#ifndef HAMMER_SCOREUPDATOR_CONFIDENCE_HPP
#define HAMMER_SCOREUPDATOR_CONFIDENCE_HPP



namespace hammer{
  namespace scoreupdator{
    template<typename Params>
    struct Confidence{
      template<typename T1>
      double operator()(const T1& t1)const
      {
	return t1.getConfidence();
      }
    };

  }
}

#endif


