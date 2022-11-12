/*
osqp-eigen的求解步骤：
1.创建求解器Solver的实例
2.设置Setting参数
3.设置Data参数
4.初始化求解器Solver
5.求解优化问题
6.提取最优解

例子：
目标：min f(x)=1/2*x1^2+x2^2-x1*x2-2*x1-6*x2
约束：x1+x2<=2
    -x1+2*x2<=2
    2*x1+x2<=3

H=[1  -1;
   -1  2]
g=[-2;
    -6]
A=[1  1;
   -1 2;
   2  1]
lower=[-inf
        -inf
        -inf]
upper=[2
       2
       3]

*/

#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Dense>

int main() {
    int numberOfVariables = 2;   //设置变量数量
    int numberOfConstraints = 3; //设置约束数量

    Eigen::SparseMatrix<double> hessian;      // H二次项系数，稀疏矩阵
    Eigen::VectorXd gradient;                 // g一次项系数，列向量
    Eigen::SparseMatrix<double> linearMatrix; // low<=Ax<=up,A矩阵，稀疏矩阵
    Eigen::VectorXd lowerBound;               //上边界,列向量
    Eigen::VectorXd upperBound;               //下边界，列向量

    //初始化H矩阵
    hessian.resize(2, 2);
    hessian.insert(0, 0) = 1;
    hessian.insert(1, 0) = -1;
    hessian.insert(0, 1) = -1;
    hessian.insert(1, 1) = 2;
    std::cout << "hessian:" << std::endl << hessian << std::endl;

    //初始化g矩阵
    gradient.resize(2);
    gradient << -2, -6;
    std::cout << "gradient:" << std::endl << gradient << std::endl;

    //初始化A矩阵
    linearMatrix.resize(3, 2);
    linearMatrix.insert(0, 0) = 1;
    linearMatrix.insert(0, 1) = 1;
    linearMatrix.insert(1, 0) = -1;
    linearMatrix.insert(1, 1) = 2;
    linearMatrix.insert(2, 0) = 2;
    linearMatrix.insert(2, 1) = 1;
    std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    //初始化lowerbound
    lowerBound.resize(3);
    lowerBound << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;
    //初始化upperbound
    upperBound.resize(3);
    upperBound << 2, 2, 3;
    std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    //开始进行osqp的求解
    // 1.创建求解器Solver的实例
    OsqpEigen::Solver solver;

    // 2.设置Setting参数
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // 3.设置Data参数 ，set the initial data of the QP solver
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(numberOfVariables);     //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(numberOfConstraints); //设置A矩阵的行数，即m
    if (!solver.data()->setHessianMatrix(hessian)) return 1;    //设置P矩阵
    if (!solver.data()->setGradient(gradient)) return 1;        //设置q or f矩阵。当没有时设置为全0向量
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1; //设置线性约束的A矩阵
    if (!solver.data()->setLowerBound(lowerBound)) return 1;                //设置下边界
    if (!solver.data()->setUpperBound(upperBound)) return 1;                //设置上边界

    // 4.初始化求解器Solver，instantiate the solver
    if (!solver.initSolver()) return 1;

    // 5.求解优化问题 solve the QP problem
    if (!solver.solve()) return 1;

    // get the controller input
    clock_t time_start = clock();
    clock_t time_end = clock();
    time_start = clock();
    // 6.提取最优解
    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution();
    time_end = clock();
    std::cout << "time use:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
}
