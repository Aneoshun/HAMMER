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
    file("complete_state.dat")
  {
  }

  Eigen::VectorXd getCurrentState(int level=0){
    if(level==0)
      return _vit;
    else
      return _pos;
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
  Eigen::VectorXd _pos;// theta , x, y
  Eigen::VectorXd _state;// vit, pos
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
    _vmot+=v/2;
    /*    if (vmot(0)>10)
      vmot(0)=10;
    if (vmot(0)<-10)
      vmot(0)=-10;

    if (vmot(1)>10)
      vmot(1)=10;
    if (vmot(1)<-10)
    vmot(1)=-10;*/

    _vit(0)= (_vmot(0) - _vmot(1)) / 500;
    _vit(1)= (_vmot(0) + _vmot(1)) / 100;

  }

  void highLevel(Eigen::VectorXd vit){
    _pos(0)+=vit(0)/10;
    if(_pos(0)>M_PI)
      _pos(0)-=2*M_PI;
    if(_pos(0)<-M_PI)
      _pos(0)+=2*M_PI;

    _pos(1)+=vit(1)*cos(_pos(0))/10;
    _pos(2)+=vit(1)*sin(_pos(0))/10;

  }


  
};




#endif
