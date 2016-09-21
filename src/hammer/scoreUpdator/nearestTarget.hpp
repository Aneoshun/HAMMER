#ifndef HAMMER_SCOREUPDATOR_NEARESTTARGET_HPP
#define HAMMER_SCOREUPDATOR_NEARESTTARGET_HPP



namespace hammer{
  namespace scoreupdator{
    template<typename Params>
    struct NearestTarget{
      template<typename T1, typename State>
      double operator()(const T1& t1, const State& target)const
      {
	return -(t1.getState()-target).cwiseProduct(Params::nearesttarget::proj()).norm(); // negative value in order to be compatible with Max selector
      }
    };
  }
}

#endif
