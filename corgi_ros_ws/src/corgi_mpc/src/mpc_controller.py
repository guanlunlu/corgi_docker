#!/usr/bin/env python3
import rospy
from corgi_ros_bridge.msg import RobotStamped
import numpy as np

from quad_mpc_3df import *

class ImpedanceParam:
    def __init__(self):
        self.M = np.array([0.652, 0.652]) # Mx, My
        self.K0 = np.array([20000, 20000]) # Kx, Ky
        self.D = np.array([400, 400]) # Dx, Dy
        self.K_pid_x = np.array([2000, 1800, 50])
        self.K_pid_y = np.array([2000, 1800, 50])

class LegState:
    def __init__(self):
        self.phi = np.array([[0], [0]])
        self.tb = np.array([[np.deg2rad(17)], [0]])
        self.pose = np.array([[0], [-0.1]])
        self.force = np.array([[0],[0]])
        self.imp_param = ImpedanceParam()
        

class RobotState:
    def __init__(self):
        self.pose = np.zeros([3,1])
        self.qwxyz = np.zeros([4,1])
        self.twist = np.zeros([6,1]) # [w;v]
        self.legs = [LegState() for _ in range(4)]

class Corgi_mpc:
    def __init__(self, dt=0.025):
        self.timer = rospy.Timer(rospy.Duration(0.025), self.loop)
        
        self.state = RobotState()
        self.state_sub = rospy.Subscriber("/robot/state", RobotStamped, self.stateCallback)
        self.force_pub = rospy.Publisher('force/force_command', RobotStamped, queue_size=1024)
        
        self.dt = dt
        
        self.mpc_param_ = None
        self.mpc_controller = None
        self.pos_param = ImpedanceParam()
        self.pos_param.K0 = np.array([1e6, 1e6])
        self.pos_param.K_pid_x = np.zeros(3)
        self.pos_param.K_pid_y = np.zeros(3)
        self.imp_param = ImpedanceParam()
        
        self.fsm = "idle"
        
        self.if_init = False
        self.ref_cnt = 0
    
    def initialize(self):
        self.reference_conf()
        self.mpc_controller = MPC(self.mpc_param_)
        self.if_init = True
    
    def reference_conf(self):
        Ns = 12  # state dim.
        Nu = 12  # input dim.
        Nt = 12  # horizon
        dt = self.dt  # control frequency
        Tsim = 20
        Nsim = math.ceil(Tsim / dt)
        xmax = np.full((Ns, 1), np.inf)
        xmin = np.full((Ns, 1), -np.inf)
        umax = np.full((Nu, 1), 4000)
        umin = np.full((Nu, 1), -4000)
        self.mpc_param_ = param(Ns, Nu, Nt, dt, xmax, xmin, umax, umin)

        # Generate Reference trajectory
        p0_ref = np.array([[0], [0], [0.21]])
        q0_wxyz_ref = np.array([[1], [0], [0], [0]])
        R0_ref = quat2rotm(q0_wxyz_ref)
        w0_ref = np.array([[0], [0], [0]])
        v0_ref = np.array([[0.000], [0], [0]])
        X0_ref = np.block([[R0_ref, p0_ref], [np.zeros([1, 3]), 1]])
        xid_ref = np.block([[w0_ref], [v0_ref]])

        X_refs = [X0_ref.copy() for _ in range(Nsim)]
        xi_refs = [xid_ref.copy() for _ in range(Nsim)]
        X = X0_ref
        for i in range(1, Nsim):
            if i > 5 / dt:
                xid_ref[3, 0] = 0.001
            xid_ref_rt = xid_ref
            Xi_ = np.block([[skew(xid_ref_rt[:3]), xid_ref_rt[3:]], [0, 0, 0, 0]])
            X = X @ sp.linalg.expm(Xi_ * dt)
            X_refs[i] = X
            xi_refs[i] = xid_ref_rt.copy()
        self.mpc_param_.X_ref = X_refs  # in SE3 form
        self.mpc_param_.xi_ref = xi_refs  # twist 6x1
    
    def stateCallback(self, data):
        self.state.pose = np.array([[data.pose.position.x], [data.pose.position.y], [data.pose.position.z]])
        self.state.qwxyz = np.array([[data.pose.orientation.w], [data.pose.orientation.x], [data.pose.orientation.y], [data.pose.orientation.z]])
        self.state.twist = np.array([[data.twist.angular.x], [data.twist.angular.y], [data.twist.angular.z],
                                     [data.twist.linear.x], [data.twist.linear.y], [data.twist.linear.z]])
        
        self.state.legs[0].tb = np.array([[data.A_LF.theta], [data.A_LF.beta]])
        self.state.legs[0].phi = np.array([[data.A_LF.motor_r.angle], [data.A_LF.motor_l.angle]])
        self.state.legs[0].pose = np.array([[data.A_LF.force.pose_x], [data.A_LF.force.pose_y]])
        self.state.legs[0].force = np.array([[data.A_LF.force.force_x], [data.A_LF.force.force_y]])
        self.state.legs[0].imp_param.M = np.array([data.A_LF.impedance.M_x, data.A_LF.impedance.M_y])
        self.state.legs[0].imp_param.K0 = np.array([data.A_LF.impedance.K0_x, data.A_LF.impedance.K0_y])
        self.state.legs[0].imp_param.D = np.array([data.A_LF.impedance.D_x, data.A_LF.impedance.D_y])
        self.state.legs[0].imp_param.K_pid_x = np.array([data.A_LF.impedance.adaptive_kp_x, 
                                                   data.A_LF.impedance.adaptive_ki_x, 
                                                   data.A_LF.impedance.adaptive_kd_x])
        self.state.legs[0].imp_param.K_pid_y = np.array([data.A_LF.impedance.adaptive_kp_y, 
                                                   data.A_LF.impedance.adaptive_ki_y, 
                                                   data.A_LF.impedance.adaptive_kd_y])
        
        self.state.legs[1].tb = np.array([[data.B_RF.theta], [data.B_RF.beta]])
        self.state.legs[1].phi = np.array([[data.B_RF.motor_r.angle], [data.B_RF.motor_l.angle]])
        self.state.legs[1].pose = np.array([[data.B_RF.force.pose_x], [data.B_RF.force.pose_y]])
        self.state.legs[1].force = np.array([[data.B_RF.force.force_x], [data.B_RF.force.force_y]])
        self.state.legs[1].imp_param.M = np.array([data.B_RF.impedance.M_x, data.B_RF.impedance.M_y])
        self.state.legs[1].imp_param.K0 = np.array([data.B_RF.impedance.K0_x, data.B_RF.impedance.K0_y])
        self.state.legs[1].imp_param.D = np.array([data.B_RF.impedance.D_x, data.B_RF.impedance.D_y])
        self.state.legs[1].imp_param.K_pid_x = np.array([data.B_RF.impedance.adaptive_kp_x, 
                                                   data.B_RF.impedance.adaptive_ki_x, 
                                                   data.B_RF.impedance.adaptive_kd_x])
        self.state.legs[1].imp_param.K_pid_y = np.array([data.B_RF.impedance.adaptive_kp_y, 
                                                   data.B_RF.impedance.adaptive_ki_y, 
                                                   data.B_RF.impedance.adaptive_kd_y])
        
        self.state.legs[2].tb = np.array([[data.C_RH.theta], [data.C_RH.beta]])
        self.state.legs[2].phi = np.array([[data.C_RH.motor_r.angle], [data.C_RH.motor_l.angle]])
        self.state.legs[2].pose = np.array([[data.C_RH.force.pose_x], [data.C_RH.force.pose_y]])
        self.state.legs[2].force = np.array([[data.C_RH.force.force_x], [data.C_RH.force.force_y]])
        self.state.legs[2].imp_param.M = np.array([data.C_RH.impedance.M_x, data.C_RH.impedance.M_y])
        self.state.legs[2].imp_param.K0 = np.array([data.C_RH.impedance.K0_x, data.C_RH.impedance.K0_y])
        self.state.legs[2].imp_param.D = np.array([data.C_RH.impedance.D_x, data.C_RH.impedance.D_y])
        self.state.legs[2].imp_param.K_pid_x = np.array([data.C_RH.impedance.adaptive_kp_x, 
                                                   data.C_RH.impedance.adaptive_ki_x, 
                                                   data.C_RH.impedance.adaptive_kd_x])
        self.state.legs[2].imp_param.K_pid_y = np.array([data.C_RH.impedance.adaptive_kp_y, 
                                                   data.C_RH.impedance.adaptive_ki_y, 
                                                   data.C_RH.impedance.adaptive_kd_y])
        
        self.state.legs[3].tb = np.array([[data.D_LH.theta], [data.D_LH.beta]])
        self.state.legs[3].phi = np.array([[data.D_LH.motor_r.angle], [data.D_LH.motor_l.angle]])
        self.state.legs[3].pose = np.array([[data.D_LH.force.pose_x], [data.D_LH.force.pose_y]])
        self.state.legs[3].force = np.array([[data.D_LH.force.force_x], [data.D_LH.force.force_y]])
        self.state.legs[3].imp_param.M = np.array([data.D_LH.impedance.M_x, data.D_LH.impedance.M_y])
        self.state.legs[3].imp_param.K0 = np.array([data.D_LH.impedance.K0_x, data.D_LH.impedance.K0_y])
        self.state.legs[3].imp_param.D = np.array([data.D_LH.impedance.D_x, data.D_LH.impedance.D_y])
        self.state.legs[3].imp_param.K_pid_x = np.array([data.D_LH.impedance.adaptive_kp_x, 
                                                   data.D_LH.impedance.adaptive_ki_x, 
                                                   data.D_LH.impedance.adaptive_kd_x])
        self.state.legs[3].imp_param.K_pid_y = np.array([data.D_LH.impedance.adaptive_kp_y, 
                                                   data.D_LH.impedance.adaptive_ki_y, 
                                                   data.D_LH.impedance.adaptive_kd_y])        
    
    def standup(self, vz=0.02):
        pass
        
    
    def loop(self, event):
        rospy.loginfo("Timer callback triggered")
        print(self.fsm)
        if self.fsm == "idle":
            pass
        
        elif self.fsm == "standup":
            if np.abs(self.state.pose[2] - self.mpc_param_.X_ref[0][2,3]) < 0.005:
                self.fsm = "mpc"
                print("switch to mpc")
            else:
                rs_ = [np.array([self.state.legs[i].pose[0,0], 0, self.state.legs[i].pose[1,0]]) + np.array([0,0,-0.05*self.dt]) for i in range(4)]
                u_ = np.ones([12,1]) * 10
                self.commandPublish(u_, rs_, self.pos_param)
                
        elif self.fsm == "mpc":
            X_ref_rt = self.mpc_param_.X_ref[self.ref_cnt]
            xi_ref_rt = self.mpc_param_.xi_ref[self.ref_cnt : (self.ref_cnt + self.mpc_param_.Nt)]
            
            X_se3_ = np.block([[quat2rotm(self.state.qwxyz), self.state.pose], [0,0,0,1]])
            rs_ = [np.array([self.state.legs[i].pose[0,0], 0, self.state.legs[i].pose[1,0]]) for i in range(4)]
            
            # print("X_se3_,\n", X_se3_)
            # print("X_ref_rt,\n", X_ref_rt)
            # print("xi_,\n", self.state.twist)
            # print("xi_ref,\n", xi_ref_rt)
            # print("rs_,\n", rs_)
            
            u = self.mpc_controller.mpc(X_se3_, X_ref_rt, self.state.twist, xi_ref_rt, rs_)
            # calculate total wrench
            b0_ = np.block([[skew(rs_[0])], [np.eye(3)]])
            b1_ = np.block([[skew(rs_[1])], [np.eye(3)]])
            b2_ = np.block([[skew(rs_[2])], [np.eye(3)]])
            b3_ = np.block([[skew(rs_[3])], [np.eye(3)]])
            b_ = np.hstack([b0_, b1_, b2_, b3_])
            u_total = b_ @ u.reshape(12,1)
            
            t_span = [0, dt]
            t_eval = np.arange(t_span[0], t_span[1] + dt / 20, dt / 20)
            x_ = np.vstack([self.state.qwxyz, self.state.pose, self.state.twist])
            sol = solve_ivp(
                SE3Dyn,
                t_span,
                x_.reshape(13),
                args=(u_total, self.mpc_param_.I),
                t_eval=t_eval,
                method="RK45",
            )
            x_next = sol.y[:, -1].reshape(-1, 1)
            X_se3_next = np.block([[quat2rotm(x_next[:4]), x_next[4:7]], [0,0,0,1]])
            rs_next = []
            for rs in rs_:
                rs_next.append(np.linalg.inv(X_se3_next)@X_se3_@rs.reshape(3,1))
            self.commandPublish(u, rs_next, self.imp_param)
            self.ref_cnt += 1
    
    def commandPublish(self, u, rs, imp_param):
        robot_msg = RobotStamped()
        robot_msg.A_LF.force.pose_x = rs[0][0]
        robot_msg.A_LF.force.pose_y = rs[0][2]
        robot_msg.A_LF.force.force_x = u[0]
        robot_msg.A_LF.force.force_y = u[2]
        
        robot_msg.A_LF.impedance.M_x = imp_param.M[0]
        robot_msg.A_LF.impedance.M_y = imp_param.M[1]
        robot_msg.A_LF.impedance.K0_x = imp_param.K0[0]
        robot_msg.A_LF.impedance.K0_y = imp_param.K0[1]
        robot_msg.A_LF.impedance.D_x = imp_param.D[0]
        robot_msg.A_LF.impedance.D_y = imp_param.D[1]
        robot_msg.A_LF.impedance.adaptive_kp_x = imp_param.K_pid_x[0]
        robot_msg.A_LF.impedance.adaptive_ki_x = imp_param.K_pid_x[1]
        robot_msg.A_LF.impedance.adaptive_kd_x = imp_param.K_pid_x[2]
        robot_msg.A_LF.impedance.adaptive_kp_y = imp_param.K_pid_y[0]
        robot_msg.A_LF.impedance.adaptive_ki_y = imp_param.K_pid_y[1]
        robot_msg.A_LF.impedance.adaptive_kd_y = imp_param.K_pid_y[2]
        
        robot_msg.B_RF.force.pose_x = rs[1][0]
        robot_msg.B_RF.force.pose_y = rs[1][2]
        robot_msg.B_RF.force.force_x = u[3]
        robot_msg.B_RF.force.force_y = u[5]
        robot_msg.B_RF.impedance.M_x = imp_param.M[0]
        robot_msg.B_RF.impedance.M_y = imp_param.M[1]
        robot_msg.B_RF.impedance.K0_x = imp_param.K0[0]
        robot_msg.B_RF.impedance.K0_y = imp_param.K0[1]
        robot_msg.B_RF.impedance.D_x = imp_param.D[0]
        robot_msg.B_RF.impedance.D_y = imp_param.D[1]
        robot_msg.B_RF.impedance.adaptive_kp_x = imp_param.K_pid_x[0]
        robot_msg.B_RF.impedance.adaptive_ki_x = imp_param.K_pid_x[1]
        robot_msg.B_RF.impedance.adaptive_kd_x = imp_param.K_pid_x[2]
        robot_msg.B_RF.impedance.adaptive_kp_y = imp_param.K_pid_y[0]
        robot_msg.B_RF.impedance.adaptive_ki_y = imp_param.K_pid_y[1]
        robot_msg.B_RF.impedance.adaptive_kd_y = imp_param.K_pid_y[2]
        
        robot_msg.C_RH.force.pose_x = rs[2][0]
        robot_msg.C_RH.force.pose_y = rs[2][2]
        robot_msg.C_RH.force.force_x = u[6]
        robot_msg.C_RH.force.force_y = u[8]
        robot_msg.C_RH.impedance.M_x = imp_param.M[0]
        robot_msg.C_RH.impedance.M_y = imp_param.M[1]
        robot_msg.C_RH.impedance.K0_x = imp_param.K0[0]
        robot_msg.C_RH.impedance.K0_y = imp_param.K0[1]
        robot_msg.C_RH.impedance.D_x = imp_param.D[0]
        robot_msg.C_RH.impedance.D_y = imp_param.D[1]
        robot_msg.C_RH.impedance.adaptive_kp_x = imp_param.K_pid_x[0]
        robot_msg.C_RH.impedance.adaptive_ki_x = imp_param.K_pid_x[1]
        robot_msg.C_RH.impedance.adaptive_kd_x = imp_param.K_pid_x[2]
        robot_msg.C_RH.impedance.adaptive_kp_y = imp_param.K_pid_y[0]
        robot_msg.C_RH.impedance.adaptive_ki_y = imp_param.K_pid_y[1]
        robot_msg.C_RH.impedance.adaptive_kd_y = imp_param.K_pid_y[2]

        robot_msg.D_LH.force.pose_x = rs[3][0]
        robot_msg.D_LH.force.pose_y = rs[3][2]
        robot_msg.D_LH.force.force_x = u[9]
        robot_msg.D_LH.force.force_y = u[11]
        robot_msg.D_LH.impedance.M_x = imp_param.M[0]
        robot_msg.D_LH.impedance.M_y = imp_param.M[1]
        robot_msg.D_LH.impedance.K0_x = imp_param.K0[0]
        robot_msg.D_LH.impedance.K0_y = imp_param.K0[1]
        robot_msg.D_LH.impedance.D_x = imp_param.D[0]
        robot_msg.D_LH.impedance.D_y = imp_param.D[1]
        robot_msg.D_LH.impedance.adaptive_kp_x = imp_param.K_pid_x[0]
        robot_msg.D_LH.impedance.adaptive_ki_x = imp_param.K_pid_x[1]
        robot_msg.D_LH.impedance.adaptive_kd_x = imp_param.K_pid_x[2]
        robot_msg.D_LH.impedance.adaptive_kp_y = imp_param.K_pid_y[0]
        robot_msg.D_LH.impedance.adaptive_ki_y = imp_param.K_pid_y[1]
        robot_msg.D_LH.impedance.adaptive_kd_y = imp_param.K_pid_y[2]
        
        self.force_pub.publish(robot_msg)
        print("cmd published")
            

if __name__ == '__main__':
    print("MPC started")
    rospy.init_node("corgi_mpc", anonymous=True)
    
    cmpc = Corgi_mpc()
    cmpc.initialize()
    cmpc.fsm = "standup"
    rospy.spin()
    