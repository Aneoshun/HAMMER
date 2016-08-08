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

  const typename boost::fusion::result_of::value_at<composition_t,boost::mpl::int_<0>  >::type& system()const
  {return boost::fusion::at<boost::mpl::int_< 0 > >(_compo);}
  
private:  


  template<int N, typename ARCH>
  class Wrapper{
  public:
    typedef typename boost::fusion::result_of::value_at<ARCH,boost::mpl::int_<0>  >::type SYS;
    typedef typename SYS::state_t state_t;
    
    Wrapper(ARCH& arch):_arch(arch){}
    
    void printStructure(){
      std::cout<<"<-------------- Layer "<<N<<" -------------->"<<std::endl;
      boost::fusion::at<boost::mpl::int_< N > >(_arch).printStructure();
      Wrapper<N-1,ARCH> p(_arch);
      p.printStructure();
    }
    
    
    state_t operator()(const typename SYS::state_t& target){
      
      std::cout<<"wrapper "<<N<<" target "<< target.transpose() <<std::endl;
      Wrapper<N-1,ARCH> p(_arch);
      boost::fusion::at<boost::mpl::int_< N > >(_arch).run(p, target, boost::fusion::at<boost::mpl::int_< 0 > >(_arch).getCurrentState(N-1));
      state_t res=boost::fusion::at<boost::mpl::int_< 0 > >(_arch).getCurrentState(N-1);
      std::cout<<"wrapper "<<N<<" target "<< target.transpose()<< "  RES: "<<res.transpose() <<std::endl;
      return boost::fusion::at<boost::mpl::int_< 0 > >(_arch).getCurrentState(N);
    }
    
    state_t getCurrentState(){return boost::fusion::at<boost::mpl::int_< 0 > >(_arch).getCurrentState(N);}
    
    ARCH& _arch;
    
  };
  
  template<typename ARCH>
  class Wrapper<0, ARCH>{
  public:
    typedef typename boost::fusion::result_of::value_at<ARCH,boost::mpl::int_<0>  >::type SYS;
    typedef typename SYS::state_t state_t;
    
    Wrapper(ARCH& arch):_arch(arch){}
    void printStructure(){
      std::cout<<"<-------------- Layer SYSTEM -------------->"<<std::endl;
    }
    
    state_t operator()(const typename SYS::state_t& target){
      //std::cout<<"wrapper SYSTEN target "<< target.transpose() <<std::endl;
      state_t res=boost::fusion::at<boost::mpl::int_< 0 > >(_arch).operator()(target);
      //std::cout<<"wrapper SYSTEM target "<< target.transpose()<< "  RES: "<<res.transpose() <<std::endl;
      return res;
    }
    
    state_t getCurrentState(){return boost::fusion::at<boost::mpl::int_< 0 > >(_arch).getCurrentState(0);}
    
    ARCH& _arch;
    
  };

  composition_t _compo;
  
};


#endif
