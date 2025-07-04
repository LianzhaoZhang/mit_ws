#ifndef MPC_H
#define MPC_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "thirdParty/quadProgpp/Array.hh"
#include "common/LowPassFilter.h"
#include <chrono>
#include <vector>
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

// Constants that don't need configuration
static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;

class State_MPC : public FSMState
{
public:
    State_MPC(CtrlComponents *ctrlComp);
    ~State_MPC();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);

private:
    // ************************** Parameter loading ************************** //
    void loadParameters();
    void initializeMatrices();
    std::string robot_type_;
    
    // ************************** Configurable parameters ************************** //
    int mpc_N;
    int nx;
    int nu;
    double speed_limx;
    double fz_max;
    double g;
    double miu;
    double d_time;
    
    // ************************** Publish for plotting ************************** // 
    ros::NodeHandle nh;
    ros::Publisher pub_euler = nh.advertise<geometry_msgs::Vector3>("euler_angles", 10);
    ros::Publisher pub_pos = nh.advertise<geometry_msgs::Vector3>("position", 10);
    ros::Publisher pub_speed = nh.advertise<geometry_msgs::Vector3>("linear_speed", 10);

    ros::Publisher pubcmd_euler = nh.advertise<geometry_msgs::Vector3>("cmd_euler_angles", 10);
    ros::Publisher pubcmd_pos = nh.advertise<geometry_msgs::Vector3>("cmd_position", 10);
    ros::Publisher pubcmd_speed = nh.advertise<geometry_msgs::Vector3>("cmd_linear_speed", 10);

    geometry_msgs::Vector3 msg_euler, msg_pos, msg_speed, cmd_euler, cmd_pos, cmd_speed;
    
    // ************************** Important parameters ************************** // 
    void setWeight();
    virtual void getUserCmd();
    void calcCmd();

    void calcTau(); 
    void calcQQd();

    void SetMatrices();
    void ConstraintsSetup();
    void solveQP();
    void calcFe();
    void Publishing();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3 _posBody, _velBody, _rpy;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;
    
    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal,current_euler;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau, tau_set;
    Vec12 _initau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;
    double dt_actual;
    // MPC用到的变量
    double _mass;       
    Eigen::Matrix<double, 5, 3> miuMat;
    Eigen::Matrix<double, 3, 3> Ic;
    Eigen::VectorXd currentStates;
    Eigen::VectorXd Xd;
    Eigen::Matrix<double, 3, 3> R_curz;
    Eigen::MatrixXd Ac;
    Eigen::MatrixXd Bc;
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd Aqp;
    Eigen::MatrixXd Bd_list;
    Eigen::MatrixXd Bqp, x_vec, prediction_X;
    Eigen::MatrixXd dense_hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd Q_diag;
    Eigen::MatrixXd R_diag;
    Eigen::MatrixXd Q_diag_N;
    Eigen::MatrixXd R_diag_N;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd CI_, CE_;
    Eigen::VectorXd ci0_, ce0_;

    Vec12 F_, Fprev;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    Eigen::Matrix<double, 3, 3> CrossProduct_A(Eigen::Matrix<double, 3, 1> A);
    Eigen::Matrix<double, 3, 3> Rz3(double theta);
};

#endif // MPC_H
