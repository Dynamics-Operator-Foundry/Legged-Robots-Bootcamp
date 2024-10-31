#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <OsqpEigen/OsqpEigen.h>
#include <chrono>

class osqpwrapper
{
private:
    // solutions
    Eigen::VectorXd _qpsol;
    std::vector<Eigen::VectorXd> _qpsol_array;

public:
    osqpwrapper(/* args */){};    
    ~osqpwrapper(){};

    void qp_opt(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _g, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb
    );

    void reset_solver();

    // solver itself
    OsqpEigen::Solver _qpsolver;

    inline Eigen::VectorXd getQpsol(){return _qpsol;}
};

#endif