#ifndef SIMPLE_FM_HPP
#define SIMPLE_FM_HPP


namespace hammer {
  namespace forwardModels{
    template<typename Action = Eigen::VectorXd , typename State= Eigen::VectorXd>
    class Simple{
    public:
      Simple(){}
      //template<typename T>
      //void predict(const T& t) const{std::cout<<"test FORWARD" << t<<std::endl;}
      State predict (const Action& action, const State& current) const{
	State a(2);
	a<<55,99;
	//State::Random(2);
	Action nn=action;
	std::cout<<"test FORWARD  received : "<<nn << "    send : "<<a<<std::endl;
	return a;
      }              
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

    private:
      

    };

  }

}




#endif
