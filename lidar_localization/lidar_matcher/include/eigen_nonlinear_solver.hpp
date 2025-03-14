#ifndef EIGEN_NONLINEAR_SOLVER_HPP  
#define EIGEN_NONLINEAR_SOLVER_HPP 
#include <unsupported/Eigen/NonLinearOptimization>
#include "se3.hpp"

Eigen::Matrix3d skew_mat(const Eigen::Vector3d& vec)
{
	Eigen::Matrix3d mat;
	mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
	return mat;
}

template <int N>
class EigenSolver
{
public:
  EigenSolver() = default;
  virtual ~EigenSolver(){};
  virtual void addData(const Target& measured_target, const Target& ref_target) = 0;
  virtual bool solve() = 0;
  // virtual double getResidual()
  // {
  //   return 0;
  // }
	virtual void getResidual(double& , std::vector<double>* ) = 0;
  virtual int getNumResiduals()
  {
    return 0;
  }
  void setInitialValue(const std::array<double, N>& initial)
  {
    for (std::size_t i = 0; i < N; i++)
    {
      para_[i] = initial[i];
    }
  }
  void setPreRotation(const Eigen::Matrix3d& rot)
  {
    pre_rotation_ = rot;
  }
  Eigen::Matrix<double, N, 1> getResult()
  {
    return Eigen::Map<Eigen::Matrix<double, N, 1>>(para_, N);
  }

protected:
  const int DIM = N;
  double para_[N];
  Eigen::Matrix3d pre_rotation_ = Eigen::Matrix3d::Identity();
};

/* Generic Eigen functor */
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct GenericFunctor
{
  typedef _Scalar Scalar;
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  GenericFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
  {
  }
  GenericFunctor(int inputs, int values) : m_inputs(inputs), m_values(values)
  {
  }

  int inputs() const
  {
    return m_inputs;
  }
  int values() const
  {
    return m_values;
  }

  // you should define that in the subclass :
  //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};


class SolvePoseFromPointToPlaneEigen : public EigenSolver<7>  
{  
public:  
  SolvePoseFromPointToPlaneEigen()  
  {  
    // 初始化平移和旋转参数  
    para_[0] = 0; // 平移 x  
    para_[1] = 0; // 平移 y  
    para_[2] = 0; // 平移 z  
    para_[3] = 0; // 四元数 x  
    para_[4] = 0; // 四元数 y  
    para_[5] = 0; // 四元数 z  
    para_[6] = 1; // 四元数 w
		for (std::size_t i = 0; i < 3; ++i) {
      para_t_[i] = 0;
      para_w_[i] = 0;
    }  
  }  

  // 添加单个点和对应的平面  
  virtual void addData(const Target& measured_target, const Target& ref_target) override  
  {  
    // 提取点和平面数据  
    Eigen::Vector3d point = measured_target.pos;  
    Eigen::Vector3d normal = ref_target.normal.normalized();  
    Eigen::Vector3d plane_point = ref_target.pos;  

    // 存储到成员变量中  
    vec_p_.push_back(point);  
    vec_plane_normal_.push_back(normal);  
    vec_plane_center_.push_back(plane_point);  
  }

	// 获取残差和最终成本  
	virtual void getResidual(double& final_cost, std::vector<double>* residuals) override {  

			// 将残差从 Eigen::VectorXd 转换为 std::vector<double>  
			residuals->resize(total_residuals_.size());  
			for (int i = 0; i < total_residuals_.size(); ++i) {  
					(*residuals)[i] = total_residuals_[i];  
			}  

			// 计算最终成本  
			final_cost = total_residuals_.squaredNorm();  
	}

  // 执行优化  
  virtual bool solve()  
  {  
    std::cout << "vec_p_ size():" << vec_p_.size() << std::endl;
		// 构造残差函数  
    Functor functor(vec_p_, vec_plane_normal_, vec_plane_center_);  
    Eigen::LevenbergMarquardt<Functor> lm(functor);

		//// 设置优化参数  
    lm.parameters.maxfev = 2000;               // 最大函数评估次数  
    lm.parameters.xtol = 1e-12;                // 参数变化阈值  
    lm.parameters.ftol = 1e-12;                // 目标函数变化阈值  
    lm.parameters.gtol = 1e-12;                // 梯度阈值  
    lm.parameters.epsfcn = 1e-16;              // 相对误差阈值  
    lm.parameters.factor = 200.0;              // 初始阻尼参数   

    // 初始化优化变量（平移 + 四元数）  
    // Eigen::VectorXd x(7);  
    // x << para_[0], para_[1], para_[2], para_[3], para_[4], para_[5], para_[6];
		Eigen::VectorXd x(6);  
    x << para_t_[0], para_t_[1], para_t_[2], para_w_[0], para_w_[1], para_w_[2];   

    // 执行优化  
    int info = lm.minimize(x);  

    // 打印优化结果  
    //std::cout << "SolvePoseFromPointToPlaneEigen:: Pose p2p solver Eigen LM status: " << info << std::endl;  
		// std::cout << "3333333333333333333333333333333333333333"<<std::endl;

    // 更新优化结果  
    // para_[0] = x(0); // 平移 x  
    // para_[1] = x(1); // 平移 y  
    // para_[2] = x(2); // 平移 z  
    // para_[3] = x(3); // 四元数 x  
    // para_[4] = x(4); // 四元数 y  
    // para_[5] = x(5); // 四元数 z  
    // para_[6] = x(6); // 四元数 w
		Eigen::Vector3d so_w;
		so_w << x[3], x[4], x[5];
		Sophus::SO3d so3 = Sophus::SO3d::exp(so_w);
		para_[0] = x[0];
		para_[1] = x[1];
		para_[2] = x[2];
		auto so3_unit_quaternion = so3.unit_quaternion();
		para_[3] = so3_unit_quaternion.x();
		para_[4] = so3_unit_quaternion.y();
		para_[5] = so3_unit_quaternion.z();
		para_[6] = so3_unit_quaternion.w();

		/////计算最终残差以及final_cost
		total_residuals_  = functor.total_func_residuals_;
		// std::cout << "total_residuals_ : \n" << total_residuals_ << std::endl;
		functor(x, total_residuals_); // 使用优化后的变量计算残差  
		final_cost_ = total_residuals_.squaredNorm(); // 最终成本为残差平方和 

		
    // return true;
		return (info == 1 || info == 2 || info == 3 || info == 4 || info ==6 || info == 7 || info ==8);
  }  

private:  
  std::vector<Eigen::Vector3d> vec_p_;             // 点云  
  std::vector<Eigen::Vector3d> vec_plane_normal_;  // 平面法向量  
  std::vector<Eigen::Vector3d> vec_plane_center_;  // 平面上的点
	double final_cost_;         // 最终成本
	Eigen::VectorXd total_residuals_; //最终所有的残差
	double para_t_[3];
  double para_w_[3];


  // 残差函数  
  struct Functor : public GenericFunctor<double>  
  {  
  public:  
    Functor(const std::vector<Eigen::Vector3d>& vec_p, const std::vector<Eigen::Vector3d>& vec_plane_normal,  
            const std::vector<Eigen::Vector3d>& vec_plane_center)  
      : GenericFunctor<double>(7, vec_p.size())  
      , vec_func_p_(vec_p)  
      , vec_func_plane_normal_(vec_plane_normal)  
      , vec_func_plane_center_(vec_plane_center)  
    {  
    }  

    // 残差计算  
    int operator()(const Eigen::VectorXd& b, Eigen::VectorXd& fvec)  
    {  
      // 提取平移和四元数  
      // Eigen::Vector3d translation= b.segment<3>(0); // 平移  
      // Eigen::Quaterniond quaternion(b[6], b[3], b[4], b[5]); // 四元数  
      // quaternion.normalize(); // 确保四元数归一化
			Eigen::Vector3d translation(b[0],b[1],b[2]); // 平移  
      Eigen::Vector3d so_w;
      so_w << b[3], b[4], b[5];
			Eigen::Matrix3d R = Sophus::SO3d::exp(so_w).matrix();   

      // 遍历每个点，计算残差  
      for (std::size_t i = 0; i < vec_func_p_.size(); i++)  
      {  
        const auto& plane_center = vec_func_plane_center_.at(i);  
        const auto& plane_normal = vec_func_plane_normal_.at(i);  
        const auto& point = vec_func_p_.at(i);  

        // 旋转点并计算残差  
        Eigen::Vector3d rotated_p = R * point; // 应用旋转  
        fvec[i] = (rotated_p + translation - plane_center).dot(plane_normal); // 点到平面的距离  
      }
			total_func_residuals_ = fvec;  
      return 0;  
    }  

    // 雅可比矩阵计算  
    int df(const Eigen::VectorXd& b, Eigen::MatrixXd& fjac)  
    {  
      // 提取平移和四元数  
      // Eigen::Vector3d translation = b.segment<3>(0); // 平移  
      // Eigen::Quaterniond quaternion(b[6], b[3], b[4], b[5]); // 四元数  
      // quaternion.normalize(); // 确保四元数归一化  
			Eigen::Vector3d translation(b[0],b[1],b[2]); // 平移  
      Eigen::Vector3d so_w;
      so_w << b[3], b[4], b[5];
			Eigen::Matrix3d R = Sophus::SO3d::exp(so_w).matrix();

      ////遍历每个点，计算雅可比矩阵  
      for (std::size_t i = 0; i < vec_func_p_.size(); i++)  
      {  
        const auto& plane_normal = vec_func_plane_normal_.at(i);  
        const auto& point = vec_func_p_.at(i);  

        // 对平移的偏导数（直接是法向量）  
        fjac.block<1, 3>(i, 0) = plane_normal.transpose();  

        // 对四元数的偏导数  
        // Eigen::Matrix3d skew_p = -Eigen::Matrix3d::Identity().cross(point);  // 叉乘矩阵  
				// Eigen::Matrix3d skew_p = skew_mat(quaternion.toRotationMatrix()*point);  // 叉乘矩阵 
				Eigen::Matrix3d skew_p = skew_mat(R * point);  // 叉乘矩阵
        fjac.block<1, 3>(i, 3) = (skew_p * plane_normal).transpose(); // 将结果赋值到雅可比矩阵  
      }  
      return 0;  
    }  

	public:
		Eigen::VectorXd total_func_residuals_;
  private:  
    std::vector<Eigen::Vector3d> vec_func_p_;             // 点云  
    std::vector<Eigen::Vector3d> vec_func_plane_normal_;  // 平面法向量  
    std::vector<Eigen::Vector3d> vec_func_plane_center_;  // 平面上的点
		 
  };  
};

#endif