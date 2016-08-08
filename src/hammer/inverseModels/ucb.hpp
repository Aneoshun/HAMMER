#ifndef UCB_IM_HPP
#define UCB_IM_HPP


#ifdef USE_NLOPT
template<typename Params, typename FM>
class UCB_FMOptimizer{
public:
  UCB_FMOptimizer(const FM& fm,size_t dim_in):_fm(fm),_dim_in(dim_in){}


  
  Eigen::VectorXd predict(const Eigen::VectorXd& current, const Eigen::VectorXd& target)
  {
    _current=current;
    _target=target;
    limbo::opt::NLOptNoGrad<ParamsNoGrad, nlopt::LN_BOBYQA> BOBYQA;
    Eigen::VectorXd res_lbfgs = BOBYQA(*this, limbo::tools::random_vector(_dim_in), false);
    return res_lbfgs;
  }
  void update(const Eigen::VectorXd& prev ,const Eigen::VectorXd& action, const Eigen::VectorXd& next){}
  

  
  limbo::opt::eval_t operator()(const Eigen::VectorXd& params, bool eval_grad = false) const
  {
    double v = -(_fm.predict(params,_current)-_target).norm() + Params::ucb::alpha() * _fm.uncertainty(params,_current);
    return limbo::opt::no_grad(v);
  }

private:  
  struct ParamsNoGrad {
    struct opt_nloptnograd {
      BO_PARAM(int, iterations, 100);
    };
  };
  
  Eigen::VectorXd _current;
  Eigen::VectorXd _target;
  const FM& _fm;
  size_t _dim_in;
  
};
#endif
#endif
