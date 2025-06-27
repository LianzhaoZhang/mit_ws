/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"
#include <ros/ros.h>
#include <string>

struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct GainConfig {
    unsigned int mode;
    double Kp;
    double Kd;
};

struct LowlevelCmd{
    MotorCmd motorCmd[12];
    
    // Configurable parameters
    Vec2 torque_limit;
    GainConfig sim_stance_hip, sim_stance_thigh, sim_stance_calf;
    GainConfig real_stance_hip, real_stance_thigh, real_stance_calf;
    GainConfig stable_hip, stable_thigh, stable_calf;
    GainConfig swing_hip, swing_thigh, swing_calf;
    
    // Constructor to load parameters
    LowlevelCmd() {
        ros::NodeHandle nh;
        std::string robot_type;
        
        if (!nh.getParam("/robot_type", robot_type)) {
            std::cout << "\033[1;31m[WARNING | LowlevelCmd] Robot type not specified, using default: go1" << "\033[0m" << std::endl;
            robot_type = "go1";
        }
        
        // Load torque limits
        std::vector<double> torque_lim_vec;
        nh.param(robot_type + "/lowlevel_cmd/torque_limit", torque_lim_vec, std::vector<double>{-50.0, 50.0});
        torque_limit = Vec2(torque_lim_vec[0], torque_lim_vec[1]);
        
        // Load sim stance gains
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/hip/mode", (int&)sim_stance_hip.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/hip/Kp", sim_stance_hip.Kp, 180.0);
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/hip/Kd", sim_stance_hip.Kd, 8.0);
        
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/thigh/mode", (int&)sim_stance_thigh.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/thigh/Kp", sim_stance_thigh.Kp, 180.0);
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/thigh/Kd", sim_stance_thigh.Kd, 8.0);
        
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/calf/mode", (int&)sim_stance_calf.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/calf/Kp", sim_stance_calf.Kp, 300.0);
        nh.param(robot_type + "/lowlevel_cmd/sim_stance_gain/calf/Kd", sim_stance_calf.Kd, 15.0);
        
        // Load real stance gains
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/hip/mode", (int&)real_stance_hip.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/hip/Kp", real_stance_hip.Kp, 60.0);
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/hip/Kd", real_stance_hip.Kd, 5.0);
        
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/thigh/mode", (int&)real_stance_thigh.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/thigh/Kp", real_stance_thigh.Kp, 40.0);
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/thigh/Kd", real_stance_thigh.Kd, 4.0);
        
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/calf/mode", (int&)real_stance_calf.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/calf/Kp", real_stance_calf.Kp, 80.0);
        nh.param(robot_type + "/lowlevel_cmd/real_stance_gain/calf/Kd", real_stance_calf.Kd, 7.0);
        
        // Load stable gains
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/hip/mode", (int&)stable_hip.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/hip/Kp", stable_hip.Kp, 0.8);
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/hip/Kd", stable_hip.Kd, 0.8);
        
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/thigh/mode", (int&)stable_thigh.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/thigh/Kp", stable_thigh.Kp, 0.8);
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/thigh/Kd", stable_thigh.Kd, 0.8);
        
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/calf/mode", (int&)stable_calf.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/calf/Kp", stable_calf.Kp, 0.8);
        nh.param(robot_type + "/lowlevel_cmd/stable_gain/calf/Kd", stable_calf.Kd, 0.8);
        
        // Load swing gains
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/hip/mode", (int&)swing_hip.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/hip/Kp", swing_hip.Kp, 3.0);
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/hip/Kd", swing_hip.Kd, 2.0);
        
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/thigh/mode", (int&)swing_thigh.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/thigh/Kp", swing_thigh.Kp, 3.0);
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/thigh/Kd", swing_thigh.Kd, 2.0);
        
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/calf/mode", (int&)swing_calf.mode, 10);
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/calf/Kp", swing_calf.Kp, 3.0);
        nh.param(robot_type + "/lowlevel_cmd/swing_gain/calf/Kd", swing_calf.Kd, 2.0);
    }

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3 qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12 qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3 qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    void setTau(Vec12 tau){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torque_limit);
        }
    }
    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].mode = sim_stance_hip.mode;
        motorCmd[legID*3+0].Kp = sim_stance_hip.Kp;
        motorCmd[legID*3+0].Kd = sim_stance_hip.Kd;
        motorCmd[legID*3+1].mode = sim_stance_thigh.mode;
        motorCmd[legID*3+1].Kp = sim_stance_thigh.Kp;
        motorCmd[legID*3+1].Kd = sim_stance_thigh.Kd;
        motorCmd[legID*3+2].mode = sim_stance_calf.mode;
        motorCmd[legID*3+2].Kp = sim_stance_calf.Kp;
        motorCmd[legID*3+2].Kd = sim_stance_calf.Kd;
    }
    void setRealStanceGain(int legID){
        motorCmd[legID*3+0].mode = real_stance_hip.mode;
        motorCmd[legID*3+0].Kp = real_stance_hip.Kp;
        motorCmd[legID*3+0].Kd = real_stance_hip.Kd;
        motorCmd[legID*3+1].mode = real_stance_thigh.mode;
        motorCmd[legID*3+1].Kp = real_stance_thigh.Kp;
        motorCmd[legID*3+1].Kd = real_stance_thigh.Kd;
        motorCmd[legID*3+2].mode = real_stance_calf.mode;
        motorCmd[legID*3+2].Kp = real_stance_calf.Kp;
        motorCmd[legID*3+2].Kd = real_stance_calf.Kd;
    }
    void setZeroGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0;
        motorCmd[legID*3+0].Kd = 0;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    void setStableGain(int legID){
        motorCmd[legID*3+0].mode = stable_hip.mode;
        motorCmd[legID*3+0].Kp = stable_hip.Kp;
        motorCmd[legID*3+0].Kd = stable_hip.Kd;
        motorCmd[legID*3+1].mode = stable_thigh.mode;
        motorCmd[legID*3+1].Kp = stable_thigh.Kp;
        motorCmd[legID*3+1].Kd = stable_thigh.Kd;
        motorCmd[legID*3+2].mode = stable_calf.mode;
        motorCmd[legID*3+2].Kp = stable_calf.Kp;
        motorCmd[legID*3+2].Kd = stable_calf.Kd;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }
    void setSwingGain(int legID){
        motorCmd[legID*3+0].mode = swing_hip.mode;
        motorCmd[legID*3+0].Kp = swing_hip.Kp;
        motorCmd[legID*3+0].Kd = swing_hip.Kd;
        motorCmd[legID*3+1].mode = swing_thigh.mode;
        motorCmd[legID*3+1].Kp = swing_thigh.Kp;
        motorCmd[legID*3+1].Kd = swing_thigh.Kd;
        motorCmd[legID*3+2].mode = swing_calf.mode;
        motorCmd[legID*3+2].Kp = swing_calf.Kp;
        motorCmd[legID*3+2].Kd = swing_calf.Kd;
    }
};

#endif  //LOWLEVELCMD_H