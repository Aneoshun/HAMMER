#ifndef CORE_PAIRINTERFACE_HPP__
#define CORE_PAIRINTERFACE_HPP__
  
  template < typename FMfun, typename Action, typename State, typename IMfun,typename ConfUpdator, typename ScoreUpdator >
  class PairInterface{
  public:
    PairInterface(FMfun fmfun, IMfun imfun):_fmfun(fmfun),_imfun(imfun),_confidence(0){}
    void operator()(const State& current){_state=_fmfun.operator()( this->getAction(), current);}
    const State& getState() const {return _state;}
    double getConfidence() const {return _confidence; }
    void udpateConfidence(const State& new_state){_confidence=_confUpdator(_confidence,new_state,_state); }
    void udpateConfidence(const State& new_state, const Action& executed){_confidence=_confUpdator(_confidence,new_state,_state,this->getAction(),executed); } 
    double getScore() const {return _score; }
    void updateScore(const State& target){_score=_scoreUpdator(*this, target); } 
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

#endif
