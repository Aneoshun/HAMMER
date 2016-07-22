#ifndef PID_IM_HPP
#define PID_IM_HPP


namespace hammer {
  namespace inverseModels{
    class PID{
    public:
      PID(const Eigen::MatrixXd& mP, const Eigen::MatrixXd& mI, const Eigen::MatrixXd& mD):
	_P(Eigen::VectorXd::Zero(mP.cols())),
	_I(Eigen::VectorXd::Zero(mP.cols())),
	_D(Eigen::VectorXd::Zero(mP.cols())),
	_mP(mP),
	_mI(mI),
	_mD(mD)
      {}
      
      Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
	
	Eigen::VectorXd error=target-current;
	_D = error-_P;
	_P = error;
	_I += error;
	Eigen::VectorXd action;
	
	action = _mP*_P + _mI*_I + _mD*_D;
	
	std::cout<<"error "<< error.transpose() <<" action: "<<action.transpose()<<std::endl;
	return action;
      }
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
      
    private:
      Eigen::VectorXd _P;
      Eigen::VectorXd _I;
      Eigen::VectorXd _D;
      Eigen::MatrixXd _mP;
      Eigen::MatrixXd _mI;
      Eigen::MatrixXd _mD;
      


    };
    
    
  }

}




#endif
