#ifndef SIMPLE_IM_HPP
#define SIMPLE_IM_HPP


namespace hammer {
  namespace inverseModels{
    template<typename Action>
    class MonoVal{
    public:
      MonoVal(Action act):_act(act){}
      Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
	return _act;
      }
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
      Action _act;
    };

    template<typename Action= Eigen::VectorXd, typename State= Eigen::VectorXd>
    class Simple{
    public:
      Simple(){}

      Action predict(const State& current, const State& target){
	Action action(2);
	action<<12,33;
	std::cout<<"test INVERSE send: "<<action<<std::endl;  
	return action;
      }
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

    private:
      

    };

  }

}




#endif
