#ifndef HAMMER_SCOREUPDATOR_NEARESTTARGET_HPP
#define HAMMER_SCOREUPDATOR_NEARESTTARGET_HPP



namespace hammer{
  namespace scoreupdator{
    template<typename Params>
    struct NearestTarget{
      template<typename T1>
      double operator()(const T1& t1)const
      {
        return -(t1.getState()-Params::hammer::target()).norm(); // negative value in order to be compatible with Max selector
      }
    };
  }
}

#endif
