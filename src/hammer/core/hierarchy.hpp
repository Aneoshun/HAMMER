#ifndef HIERARCHY_HPP__
#define HIERARCHY_HPP__

#include <boost/mpl/int.hpp>

#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/sequence/intrinsic/at_c.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/fusion/iterator/advance.hpp>
#include <boost/fusion/include/advance.hpp>



template<typename state_t>
struct ParseHierarchy
{
  typedef state_t result_type;

  template<typename layer_t>
  state_t operator()(const state_t& target, const layer_t& layer) const
  {
    layer.run(target);
  }
};








template<size_t L,typename Composition>
class Hierarchy{
public:
  Hierarchy(){}

  typedef typename boost::mpl::if_<boost::fusion::traits::is_sequence<Composition>, Composition, boost::fusion::vector<Composition> >::type composition_t;

  
  
  void printStructure(){
    Wrapper<L-1,composition_t> p(_compo);
    p.printStructure();
  }


  /*  template<int N, typename T, typename... Targs>
  void bindInverseModel( T& t, Targs&... tt )
    {
      boost::fusion::at<boost::mpl::int_< N > >(_compo).bindInverseModel(t, tt);
    }
  */

  template<int N, typename... Targs>
  void bindInverseModel(Targs&... tt )
  {
    boost::fusion::at<boost::mpl::int_< N > >(_compo).bindInverseModel(tt...);
  }

  template<int N, typename... Targs>
  void bindForwardModel(Targs&... tt )
  {
    boost::fusion::at<boost::mpl::int_< N > >(_compo).bindForwardModel(tt...);
  }
  

  
  template< typename state_t>
  void run( const state_t& target){
    Wrapper<L-1,composition_t> p(_compo);
    p(target);
  }

private:  


  template<int N, typename HMR>
class Wrapper{
public:
    typedef typename boost::fusion::result_of::value_at<HMR,boost::mpl::int_<0>  >::type SYS;
    typedef typename SYS::state_t state_t;
  
  Wrapper(HMR& hammer):_hmr(hammer){}

    void printStructure(){
      std::cout<<"<-------------- Layer "<<N<<" -------------->"<<std::endl;
      boost::fusion::at<boost::mpl::int_< N > >(_hmr).printStructure();
      Wrapper<N-1,HMR> p(_hmr);
      p.printStructure();
    }

    
  state_t operator()(const typename SYS::state_t& target){

    std::cout<<"wrapper target "<< target.transpose() <<std::endl;
    Wrapper<N-1,HMR> p(_hmr);
    boost::fusion::at<boost::mpl::int_< N > >(_hmr).run(p, target, boost::fusion::at<boost::mpl::int_< 0 > >(_hmr).getCurrentState(N-1));
    return boost::fusion::at<boost::mpl::int_< 0 > >(_hmr).getCurrentState(N);
  }
  
    state_t getCurrentState(){return boost::fusion::at<boost::mpl::int_< 0 > >(_hmr).getCurrentState(N);}
  
  HMR& _hmr;

};

template<typename HMR>
class Wrapper<0, HMR>{
public:
  typedef typename boost::fusion::result_of::value_at<HMR,boost::mpl::int_<0>  >::type SYS;
  typedef typename SYS::state_t state_t;
  
  Wrapper(HMR& hammer):_hmr(hammer){}
  void printStructure(){
    std::cout<<"<-------------- Layer SYSTEM -------------->"<<std::endl;
  }

  state_t operator()(const typename SYS::state_t& target){
    return boost::fusion::at<boost::mpl::int_< 0 > >(_hmr).operator()(target);
  }
  
  state_t getCurrentState(){return boost::fusion::at<boost::mpl::int_< 0 > >(_hmr).getCurrentState(0);}
  
  HMR& _hmr;

};


  

  
  composition_t _compo;
  
};


#endif
