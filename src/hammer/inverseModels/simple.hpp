#ifndef SIMPLE_IM_HPP
#define SIMPLE_IM_HPP


namespace hammer {
  namespace inverseModels{
    
    template<int val>
    class MonoVal{
    public:
      Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target){
	Eigen::VectorXd a(1);
	a<<val;
	return a;
      }
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

    };

    template<typename Action= Eigen::VectorXd, typename State= Eigen::VectorXd>
    class Simple{
    public:
      Simple(){}
      ~Simple(){std::cout<<"dead"<<std::endl;}

      Action predict(const State& current, const State& target){
	Action a(2);
	a<<12,33;
	std::cout<<"test INVERSE send: "<<a<<std::endl;  
	return a;
      }
      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}

    private:
      

    };

  }

}




#endif
