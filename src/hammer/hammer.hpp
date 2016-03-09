#ifndef HAMMER_HPP
#define HAMMER_HPP

#include <fstream>



#include <functional>

#include <boost/parameter.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Core>
#include "tbb/tbb.h"
#include "tbb/blocked_range.h"



#include <hammer/selector/max.hpp>
#include <hammer/confidUpdator/distance.hpp>
#include <hammer/scoreUpdator/confidence.hpp>
#include <hammer/stop/chain_criteria.hpp>
#include <hammer/stop/max_iterations.hpp>
#include <hammer/tools/macros.hpp>
#include <hammer/tools/sys.hpp>
#include <hammer/stat.hpp>


namespace hammer {

  namespace defaults {
    struct hammer {
      HMR_PARAM(bool, stats_enabled, true);
    };
  }


  template <typename Hammer>
  struct RefreshStat_f {
    RefreshStat_f( Hammer& hammer)
      : _hammer(hammer){}

     Hammer& _hammer;
    
    template <typename T>
    void operator()( T& x) const { x(_hammer); }
  };



  
  template < typename IMfun, typename Action, typename State>
  class IMInterface{
  public:
    IMInterface(IMfun imfun):_imfun(imfun){} 
    void operator()(const State& current, const State& target){_action=_imfun.operator()(current,target);}
    const Action& getAction() const{return _action;}
  private:
    IMfun _imfun;
    Action _action;
  };

  
  template < typename FMfun, typename Action, typename State, typename IMfun,typename ConfUpdator, typename ScoreUpdator >
  class PairInterface{
  public:
    PairInterface(FMfun fmfun, IMfun imfun):_fmfun(fmfun),_imfun(imfun),_confidence(0){}
    void operator()(const State& current){_state=_fmfun.operator()( this->getAction(), current);}//_imfun.operator()(), current);}
    const State& getState() const {return _state;}
    double getConfidence() const {return _confidence; }
    void udpateConfidence(const State& state){_confidence=_confUpdator(state,_state,_confidence); } 
    double getScore() const {return _score; }
    void updateScore(){_score=_scoreUpdator(*this); } 
    const Action& getAction() const{return _imfun.operator()();}
  private:
    FMfun _fmfun;
    State _state;
    IMfun _imfun;
    double _confidence;
    double _score;
    ConfUpdator _confUpdator;
    ScoreUpdator _scoreUpdator;
  };





  BOOST_PARAMETER_TEMPLATE_KEYWORD(state)  //1
  BOOST_PARAMETER_TEMPLATE_KEYWORD(action)  //2
  BOOST_PARAMETER_TEMPLATE_KEYWORD(confidfun)  //3
  BOOST_PARAMETER_TEMPLATE_KEYWORD(scorefun)  //4
  BOOST_PARAMETER_TEMPLATE_KEYWORD(selectfun)  //5
  BOOST_PARAMETER_TEMPLATE_KEYWORD(statsfun)  //6
  BOOST_PARAMETER_TEMPLATE_KEYWORD(stopcrit)  //7
  typedef boost::parameter::parameters<boost::parameter::optional<tag::state>,
				       boost::parameter::optional<tag::action>,
				       boost::parameter::optional<tag::confidfun>,
				       boost::parameter::optional<tag::scorefun>,
				       boost::parameter::optional<tag::selectfun>,
				       boost::parameter::optional<tag::statsfun>,
				       boost::parameter::optional<tag::stopcrit> > class_signature;
//  template< typename Params, typename State = Eigen::VectorXd, typename Action = Eigen::VectorXd,typename Selector = ConfSelector<Params>, typename ConfUpdator=Dist>
  template <class Params,
	    class A1 = boost::parameter::void_,
	    class A2 = boost::parameter::void_,
	    class A3 = boost::parameter::void_,
	    class A4 = boost::parameter::void_,
	    class A5 = boost::parameter::void_,
	    class A6 = boost::parameter::void_,
	    class A7 = boost::parameter::void_>
  class Hammer{
  public:
    typedef Params params_t;
    // defaults                                                                                                                                                             
    struct defaults {
      typedef Eigen::VectorXd state_t; // 1                                                                                                                   
      typedef Eigen::VectorXd action_t; // 2
      typedef confidupdator::Distance<Params> confidfun_t; // 3
      typedef scoreupdator::Confidence<Params> scorefun_t; // 4
      typedef selector::Max<Params> selectfun_t; // 5
      typedef boost::fusion::vector<stat::Predictions<Params>, stat::SelectedActions<Params>, stat::States<Params> > stat_t; // 6                               
      typedef boost::fusion::vector< stop::MaxIterations<Params> > stop_t; // 7
    };

    // extract the types                                                                                                                                                    
    typedef typename class_signature::bind<A1, A2, A3, A4, A5, A6, A7>::type args;
    typedef typename boost::parameter::binding<args, tag::state, typename defaults::state_t>::type state_t;
    typedef typename boost::parameter::binding<args, tag::action, typename defaults::action_t>::type action_t;
    typedef typename boost::parameter::binding<args, tag::confidfun, typename defaults::confidfun_t>::type confidfun_t;
    typedef typename boost::parameter::binding<args, tag::scorefun, typename defaults::scorefun_t>::type scorefun_t;
    typedef typename boost::parameter::binding<args, tag::selectfun, typename defaults::selectfun_t>::type selectfun_t;
    typedef typename boost::parameter::binding<args, tag::statsfun, typename defaults::stat_t>::type Stat;
    typedef typename boost::parameter::binding<args, tag::stopcrit, typename defaults::stop_t>::type StoppingCriteria;
    
    typedef typename boost::mpl::if_<boost::fusion::traits::is_sequence<StoppingCriteria>, StoppingCriteria, boost::fusion::vector<StoppingCriteria>>::type stopping_criteria_t;
    typedef typename boost::mpl::if_<boost::fusion::traits::is_sequence<Stat>, Stat, boost::fusion::vector<Stat>>::type stat_t;


    typedef std::function<action_t(state_t,state_t)> IM_t;
    typedef std::function<state_t(action_t,state_t)> FM_t;
    typedef std::function<void(state_t,action_t,state_t)> Model_t;
    typedef std::function<const action_t&()> ActionReader_t;
    typedef PairInterface<FM_t,action_t, state_t, ActionReader_t, confidfun_t, scorefun_t> ModelPair_t;    
    typedef IMInterface<IM_t, action_t, state_t> IM_ITF_t;


    Hammer():_current_iteration(0) { _make_res_dir(); }

    // disable copy (dangerous and useless)                                                                                                                                 
    Hammer(const Hammer& other) = delete;
    Hammer& operator=(const Hammer& other) = delete;

    bool stats_enabled() const { return Params::hammer::stats_enabled(); }

    const std::string& res_dir() const { return _res_dir; }

  
    template<typename System>
    void run( System& system, const state_t& target,const state_t& initState)
    {
      /*  assert(_inverseModels.size()!=0 && _forwardModels.size()!=0);
      this->_current_iteration=0;
      _currentState=initState;
      while (!this->_stop(*this)) {
	this->_update_inversePredictions(_currentState,target);
	this->_update_forwardPredictions(_currentState);
	_selectedAction=_selectAction()->getAction();
	state_t newState=system(_selectedAction); 
	this->_print_confidencePrediction();
	this->_update_confidence(newState);
	this->_update_models(_currentState, _selectedAction, newState);
	_currentState=newState;
	this->_current_iteration++;
	this->_update_stats(*this);
	}*/


      assert(_inverseModels.size()!=0 && _forwardModels.size()!=0);
      this->_current_iteration=0;
      _currentState=initState;  

      while (!this->_stop(*this)) {
	this->print_prediction();
	state_t newState=system(this->suggestAction(target,this->_currentState)); 
	updateModels(newState);
	this->_current_iteration++;
        this->_update_stats(*this);     
      }

    }
    
    template<typename System>
    void learn( System& system, const state_t& target, const state_t& initState) // IMITATION LEARNING -->>> EXPERIMENTAL
    {

      assert(_inverseModels.size()!=0 && _forwardModels.size()!=0);
      this->_current_iteration=0;
      
      //state_t target; // target or not a target that is the question
      _currentState=initState;
      while (!this->_stop(*this)) {
	this->_update_inversePredictions(_currentState,target);
	this->_update_forwardPredictions(_currentState);
	//action_t selectedAction; // same here, how do we know the executed action;
	state_t newState=system(); 
	this->print_prediction();
	this->_update_confidence(newState);
	//this->_update_models(_currentState, _selectedAction, _newState);
	_currentState=newState;
	this->_current_iteration++;
	this->_update_stats(*this);
      }
    }

     const action_t& suggestAction(const state_t& target, const state_t& currentState)
    {
      _currentState=currentState;
      assert(_inverseModels.size()!=0 && _forwardModels.size()!=0);
      this->_update_inversePredictions(currentState,target);
      this->_update_forwardPredictions(currentState);
      _selectedPair=_selectAction();
      return _selectedPair->getAction();
    }
    const std::shared_ptr<ModelPair_t> & getSelectedPair()const
    {return _selectedPair;}

    void updateModels(const state_t& newState)
    {
      this->_update_confidence(newState);
      this->_update_models(_currentState, _selectedPair->getAction(), newState);
      _currentState=newState;
    }

    template<typename T, typename... Targs>
    void bindInverseModel( T& t,  Targs&... tt )
    {
      bindInverseModel(t);
      bindInverseModel(tt...);
    }

    template<typename T>
    void bindInverseModel(T& t){
      _imFun.push_back(std::bind(&T::predict, &t,std::placeholders::_1,std::placeholders::_2));
      _inverseModels.push_back( std::shared_ptr<IM_ITF_t >( new IM_ITF_t(_imFun[_imFun.size()-1] ) ));
      _models.push_back(std::bind(&T::update, &t,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
      for(auto fm:_fmFun)
	_forwardModels.push_back( std::shared_ptr<ModelPair_t >( new ModelPair_t(fm ,std::bind(&IM_ITF_t::getAction, _inverseModels[_inverseModels.size()-1])))); 
    }

    template<typename T, typename... Targs>
    void bindForwardModel( T& t,  Targs&... tt )
    {
      bindForwardModel(t);
      bindForwardModel(tt...);
    }

    template<typename T>
    void bindForwardModel( T& t){
      _fmFun.push_back(std::bind(&T::predict, &t, std::placeholders::_1, std::placeholders::_2));
      _models.push_back(std::bind(&T::update, &t,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
      for (auto& im: _inverseModels)
      	_forwardModels.push_back(std::shared_ptr<ModelPair_t >( new ModelPair_t(_fmFun[_fmFun.size()-1], std::bind(&IM_ITF_t::getAction,im) )));
    }

    void printStructure()const
    {
      std::cout<<"IM fun: "<< _imFun.size()<<std::endl;
      std::cout<<"FM fun: "<< _fmFun.size()<<std::endl;
      std::cout<<"inverseModels fun: "<< _inverseModels.size()<<std::endl;
      std::cout<<"forwardModels fun: "<< _forwardModels.size()<<std::endl;

    }

    int current_iteration() const { return _current_iteration; }
    const std::vector< std::shared_ptr<ModelPair_t> >& getPairInterfaces() const {return _forwardModels;}
    state_t getCurrentState()const {return _currentState;}
    action_t getSelectedAction()const {return _selectedPair->getAction();}

    void print_prediction()const
    {
      std::for_each(_forwardModels.begin(),_forwardModels.end(), [=](const std::shared_ptr<ModelPair_t>& fm ) {std::cout<<"Conf: "<<(*fm).getConfidence()<<"  score: "<<(*fm).getScore()<< "  Pred: "<<(*fm).getState()<<std::endl;} );
    }


  private:

    template <typename HMR>
    bool _stop(const HMR& hmr) const
    {
      hammer::stop::ChainCriteria<HMR> chain(hmr);
      return boost::fusion::accumulate(_stopping_criteria, false, chain);
    }

    template <typename HMR>
    void _update_stats(HMR& hmr) // is not const as the stat objects modify themselves when creating the logfile.
    { 
      boost::fusion::for_each(_stat, RefreshStat_f<HMR>(hmr));
    }

    void _make_res_dir()
    {
      if (!Params::hammer::stats_enabled())
	return;
      _res_dir = tools::hostname() + "_" + tools::date() + "_" + tools::getpid();
      boost::filesystem::path my_path(_res_dir);
      boost::filesystem::create_directory(my_path);
    }

    void _update_inversePredictions(const state_t& current, const state_t& target) 
    { 
      tbb::parallel_for_each(_inverseModels.begin(),_inverseModels.end(), [=](std::shared_ptr<IM_ITF_t > & im) {(*im)(current,target);} );
    }

    void _update_forwardPredictions(const state_t& current) 
    {
      tbb::parallel_for_each(_forwardModels.begin(),_forwardModels.end(), [=](std::shared_ptr<ModelPair_t >& fm ) {(*fm)(current); (*fm).updateScore();} );
    }

    typename std::shared_ptr<ModelPair_t> _selectAction() const { return _selector(_forwardModels);}

    void _update_confidence(const state_t& state)
    {
      tbb::parallel_for_each(_forwardModels.begin(),_forwardModels.end(), [=](std::shared_ptr<ModelPair_t >& fm ) {(*fm).udpateConfidence(state);} );
    }


    void _update_models(const state_t&  prevstate_t, const action_t& action, const state_t& newstate_t)
    {
      tbb::parallel_for_each(_models.begin(),_models.end(), [=](Model_t& fm ) {fm(prevstate_t,action,newstate_t);} );
    }

    
    

    std::string _res_dir;
    int _current_iteration;
    
    selectfun_t _selector;
    stopping_criteria_t _stopping_criteria;
    stat_t _stat;
    state_t _currentState;
    std::shared_ptr<ModelPair_t>  _selectedPair;
    std::vector<std::shared_ptr<IM_ITF_t > > _inverseModels;  //shared ptr are necessary for the bindings: the ref of the vector changes when pushing elements
    std::vector< std::shared_ptr<ModelPair_t> > _forwardModels;
    std::vector<FM_t> _fmFun;
    std::vector<IM_t> _imFun;
    std::vector<Model_t> _models;
    
    
  };

  

}
#endif
