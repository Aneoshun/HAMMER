#ifndef LINEARLEASTSQUARE_FM_HPP
#define LINEARLEASTSQUARE_FM_HPP

#include<Eigen/Core>
#include <Eigen/LU>
#include<Eigen/SVD>

namespace hammer {
  namespace forwardModels{


    class LinearLeastSquare{
    public:
      LinearLeastSquare(){}

      Eigen::VectorXd predict (const Eigen::VectorXd& action, const Eigen::VectorXd& current) const{
	Eigen::VectorXd x(current.rows() + action.rows());
	x<<current,action;
	Eigen::VectorXd state=Eigen::VectorXd::Zero(current.rows());
	if (X.rows()>10){
	  state=B.transpose()*x;
	}

	return state+current;
      }              

      void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){

	Eigen::VectorXd x(prev.rows() + action.rows());
	x<<prev,action;

	X.conservativeResize(X.rows()+1, x.rows());
	X.row(X.rows()-1)=x.transpose();
	Y.conservativeResize(Y.rows()+1, next.rows());
	Y.row(Y.rows()-1)=next.transpose()-prev.transpose();
	//_compute_B();  /// DOES NOT WORKS :(
	B=(X.transpose()*X+Eigen::MatrixXd::Random(X.cols(),X.cols())*1e-10);
	B=B.inverse()*X.transpose()*Y;
	//std::cout<<B<<std::endl;
	//std::cout<<X<<std::endl;
	//std::cout<<Y<<std::endl;
	
      }

    private:
      Eigen::MatrixXd X;
      Eigen::MatrixXd Y;
      Eigen::MatrixXd B;



      void _compute_B()
      {

	B=X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
	return;
	Eigen::JacobiSVD< Eigen::MatrixXd > svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = 1e-10 * std::max(X.cols(), X.rows()) *svd.singularValues().array().abs()(0);
	B= (svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint())*Y;
      }    

      
    };

  }

}




#endif
