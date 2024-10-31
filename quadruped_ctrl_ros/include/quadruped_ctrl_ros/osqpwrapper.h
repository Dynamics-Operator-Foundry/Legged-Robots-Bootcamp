#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <OsqpEigen/OsqpEigen.h>

class osqpwrapper
{
private:
    // solver itself
    OsqpEigen::Solver _qpsolver;

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

    inline Eigen::VectorXd getQpsol(){return _qpsol;}
};

#endif