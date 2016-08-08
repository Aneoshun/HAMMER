#ifndef FMOPTIMIZER_IM_HPP
#define FMOPTIMIZER_IM_HPP

template<typename FM>
class FMOptimizer{
public:
  FMOptimizer(const FM& fm,size_t dim_in):_fm(fm),_dim_in(dim_in){}


  
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target)
  {
    limbo::opt::NLOptNoGrad<ParamsNoGrad,  nlopt::LN_BOBYQA> BOBYQA;
    Eigen::VectorXd res = BOBYQA(F(current,target,_fm), limbo::tools::random_vector(_dim_in), false);
    return res;
  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
  
  struct F{
    F(const Eigen::VectorXd& current, const Eigen::VectorXd& target, const FM& fm):_current(current),_target(target),_fm(fm){}
    limbo::opt::eval_t operator()(const Eigen::VectorXd& params, bool eval_grad = false) const
    {
      double v = -(_fm.predict(params,_current)-_target).norm();
      return limbo::opt::no_grad(v);
    }
    Eigen::VectorXd _current;
    Eigen::VectorXd _target;
    const FM& _fm;
  };
  

private:  
  struct ParamsNoGrad {
    struct opt_nloptnograd {
      BO_PARAM(int, iterations, 100);
    };
  };
  
  const FM& _fm;
  size_t _dim_in;
  
};
#endif
