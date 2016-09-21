#ifndef VEHICLE_HPP
#define VEHICLE_HPP

class Vehicle{
public:
  typedef Eigen::VectorXd state_t; 
  Vehicle():
    _vmot(Eigen::VectorXd::Zero(2)),
    _vit(Eigen::VectorXd::Zero(2)),
    _pos(Eigen::VectorXd::Zero(3)),
    _state(Eigen::VectorXd::Zero(5)),
    _theta(0),
    file("complete_state.dat")
  {
  }

  Eigen::VectorXd getCurrentState(int level=0){
    if(level==0)
      return _vit;
    else if(level==1)
      return _state;
    else
      return _pos.tail(2);
  }
  
  Eigen::VectorXd operator()(Eigen::VectorXd v){
    complete_system(v);
    //lowLevel(v);
    //highLevel(v);

    update_state();
    std::cout<<"State: "<<_state.transpose()<<std::endl;
    file<<_state.transpose()<<std::endl;

    
    return getCurrentState(0);
  }



  
private:
  Eigen::VectorXd _vmot; //vmot1, vmot2
  Eigen::VectorXd _vit; //theta p, v point
  Eigen::VectorXd _pos;// cos theta, sin theta , x, y
  Eigen::VectorXd _state;// vit, pos
  double _theta;
  std::ofstream file;

  void update_state(){
    _state(0)=_vit(0);
    _state(1)=_vit(1);
    _state(2)=_pos(0);
    _state(3)=_pos(1);
    _state(4)=_pos(2);

  }
  
  void complete_system(Eigen::VectorXd v){
    lowLevel(v);
    highLevel(_vit);
  }

  
  void lowLevel(Eigen::VectorXd v){
    _vmot+=v/10;
    if (_vmot(0)>1)
      _vmot(0)=1;
    if (_vmot(0)<-1)
      _vmot(0)=-1;

    if (_vmot(1)>1)
      _vmot(1)=1;
    if (_vmot(1)<-1)
      _vmot(1)=-1;

    
    _vit(0)= (_vmot(0) - _vmot(1));
    _vit(1)= (_vmot(0) + _vmot(1));

  }

  void highLevel(Eigen::VectorXd vit){
    _theta+=vit(0)/100;
    if(_theta>M_PI)
      _theta-=2*M_PI;
    if(_theta<-M_PI)
      _theta+=2*M_PI;

    _pos(0)=_theta;
    _pos(1)+=vit(1)*cos(_theta)/25;
    _pos(2)+=vit(1)*sin(_theta)/25;
  }


  
};




#endif
