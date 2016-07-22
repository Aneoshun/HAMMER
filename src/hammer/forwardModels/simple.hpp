#ifndef SIMPLE_FM_HPP
#define SIMPLE_FM_HPP


namespace hammer {
  namespace forwardModels{

    template<typename Action = Eigen::VectorXd , typename State= Eigen::VectorXd>
    class Simple{
    public:
      Simple(){}

      State predict (const Action& action, const State& current) const{
	State state(2);
	state<<55,99;
	
	std::cout<<"test FORWARD  received : "<<action << "    send : "<<state<<std::endl;
	return state;
      }              

      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
    };

  }

}




#endif
