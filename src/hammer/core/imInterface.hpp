#ifndef CORE_IMINTERFACE_HPP__
#define CORE_IMINTERFACE_HPP__  
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

#endif
