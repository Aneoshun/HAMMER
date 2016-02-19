#ifndef HAMMER_SELECTOR_NEARESTTARGET_HPP
#define HAMMER_SELECTOR_NEARESTTARGET_HPP



namespace hammer{
  namespace selector{
    template<typename Params>
    struct NearestTarget{

      template<typename T1, typename T2>// TODO Add aggregator function not specific to Eigen::VectorXd
      bool operator()(const T1& t1, const T2& t2)const
      {
	return -(t1->getState()-Params::hammer::target()).norm() < - (t2->getState()-Params::hammer::target()).norm();
      }
    };
  }
}

#endif
