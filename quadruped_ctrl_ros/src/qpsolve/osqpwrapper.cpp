#include "quadruped_ctrl_ros/osqpwrapper.h"

void osqpwrapper::qp_opt(
    Eigen::MatrixXd _MQM, 
    Eigen::MatrixXd _g,
    Eigen::MatrixXd _A, 
    Eigen::MatrixXd _lb, 
    Eigen::MatrixXd _ub
)
{
    // Load problem data
    
    int nV = _MQM.rows();
    int nC = _A.rows();
    
    Eigen::MatrixXd H;
    H.resize(nV, nV);
    H << _MQM;

    Eigen::VectorXd g;
    g.resize(nV);
    g << _g;

    Eigen::MatrixXd A;
    A.resize(nC, nV);
    A << _A;

    Eigen::VectorXd lb, ub;
    lb.resize(nC);
    ub.resize(nC);
    lb << _lb; //-OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    ub << _ub; //2, 2, 3;

    Eigen::SparseMatrix<double> Hessian = H.sparseView();
    Eigen::SparseMatrix<double> ALinear = A.sparseView();
    
    _qpsolver.settings()->setWarmStart(true);
    _qpsolver.settings()->setVerbosity(false);
    // _qpsolver.settings()->setMaxIteration(10);

    _qpsolver.data()->setNumberOfVariables(nV);
    _qpsolver.data()->setNumberOfConstraints(nC);

    if(!_qpsolver.data()->setHessianMatrix(Hessian))
        std::cout<<"Hessian not set!"<<std::endl;

    if(!_qpsolver.data()->setGradient(g))
        std::cout<<"gradient not set!"<<std::endl;
    
    if(!_qpsolver.data()->setLinearConstraintsMatrix(ALinear))
        std::cout<<"linear matrix not set!"<<std::endl;
    
    if(!_qpsolver.data()->setLowerBound(lb))
        std::cout<<"lb not set!!"<<std::endl;
    
    if(!_qpsolver.data()->setUpperBound(ub))
        std::cout<<"ub not set!"<<std::endl;
    
    if(!_qpsolver.initSolver())
        std::cout<<"please initialize solver!!"<<std::endl;

    if(!_qpsolver.updateHessianMatrix(Hessian))
        std::cout<<"Hessian not set!"<<std::endl;
    
    if(!_qpsolver.updateLinearConstraintsMatrix(ALinear))
        std::cout<<"linear matrix not set!"<<std::endl;
    
    OsqpEigen::ErrorExitFlag result = _qpsolver.solveProblem();
    if(result != OsqpEigen::ErrorExitFlag::NoError)
        std::cout<<"not yet solved"<<std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    _qpsol = _qpsolver.getSolution();
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed = end - start;
    // std::cout << "Computation time: " << elapsed.count() << " ms" << std::endl;

    reset_solver();
}

void osqpwrapper::reset_solver()
{
    _qpsolver.clearSolver();
    // _qpsolver
    _qpsolver.data()->clearHessianMatrix();
    _qpsolver.data()->clearLinearConstraintsMatrix();
    // _qpsolver.data()->c;

}
