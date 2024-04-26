from util import *
import osqp
import math
from matplotlib import pyplot as plt
from scipy.integrate import solve_ivp
from scipy import sparse
import sys
import pandas as pd
from logger import *
import time


class param:
    def __init__(self, Nx, Nu, Nt, dt, xmax, xmin, umax, umin):
        self.Nx = Nx  # dim. x
        self.Nu = Nu  # dim. u
        self.Nt = Nt  # horizon length
        self.dt = dt  # step time
        self.xmax = xmax  # state constraints
        self.xmin = xmin
        self.umax = umax  # input constraints
        self.umin = umin
        self.I = None  # body inertial tensor

        self.X_ref = None  # reference trajectory SE3
        self.xi_ref = None  # reference twist

        self.Q = np.diag([1, 1, 1, 5, 5, 5, 1, 1, 1, 0.5, 0.5, 0.5]) * 80
        self.P = np.diag([1, 1, 1, 10, 10, 10, 1, 1, 1, 1, 1, 1]) * 80
        self.R = np.eye(Nu) * 1e-5


class MPC:
    def __init__(self, param_):
        self.param = param_
        self.m = 20
        self.g = -9.81
        self.l = 577.5 * 0.001
        self.w = 329.5 * 0.001
        self.h = 144 * 0.001
        self.Ixx = 1 / 12 * self.m * (self.w**2 + self.h**2)
        self.Iyy = 1 / 12 * self.m * (self.l**2 + self.h**2)
        self.Izz = 1 / 12 * self.m * (self.l**2 + self.w**2)

        self.Ib = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.Mb = np.diag([self.m, self.m, self.m])
        self.Jb = np.block([[self.Ib, np.zeros((3, 3))], [np.zeros((3, 3)), self.Mb]])
        # self.Jb = np.block([[self.Ib, np.zeros((3, 3))], [np.zeros((3, 3)), np.eye(3)]])
        self.param.I = self.Jb

        self.solver = None
        self.logger = logger()

    def mpc(self, X0, Xd, xi0, xids, rs):
        # X0, Xd in SE3
        # xi0, current body twist 6x1
        # rs, leverages of each leg

        Xe = np.linalg.inv(Xd) @ X0  # 4x4
        xe = sp.linalg.logm(Xe)  # 4x4 in se3
        w_hat = xe[:3, :3]

        # state [err; twist]
        p0 = np.array(
            [w_hat[2, 1], w_hat[0, 2], w_hat[1, 0], xe[0, 3], xe[1, 3], xe[2, 3]]
        ).reshape(6, 1)
        p0 = np.vstack([p0, xi0])  # 12x1

        A, bmin, bmax = self.mpc_constraint(xids, p0, xi0, X0, rs)

        Ad = -A[: self.param.Nx, : self.param.Nx]
        Bd = -A[
            : self.param.Nx,
            (self.param.Nt + 1) * self.param.Nx : (self.param.Nt + 1) * self.param.Nx
            + self.param.Nu,
        ]

        """ eigenvalues, eigenvectors = sp.linalg.eig(A)

        # Check each eigenvalue for stabilizability
        n = Ad.shape[0]
        stabilizable = True
        for i, lambda_ in enumerate(eigenvalues):
            if np.abs(lambda_) >= 1:  # Check for unstable eigenvalues
                # Construct the controllability matrix for the eigenvalue lambda_
                controllability_matrix = Bd
                for j in range(1, n):
                    controllability_matrix = np.hstack(
                        (
                            controllability_matrix,
                            np.linalg.matrix_power(Ad - lambda_ * np.eye(n), j) @ Bd,
                        )
                    )

                # Check if the controllability matrix has full row rank
                if np.linalg.matrix_rank(controllability_matrix) < n:
                    print(
                        f"Mode associated with eigenvalue {lambda_} is not stabilizable."
                    )
                    stabilizable = False

        if stabilizable:
            print("The system is stabilizable.")
        else:
            print("The system is not fully stabilizable.") """

        # adf = pd.DataFrame(Ad)
        # bdf = pd.DataFrame(Bd)
        # adf.to_csv("./Matrices/Ad.csv", index=False, header=False)
        # bdf.to_csv("./Matrices/Bd.csv", index=False, header=False)

        self.param.P = sp.linalg.solve_discrete_are(
            -A[: self.param.Nx, : self.param.Nx],
            -A[
                : self.param.Nx,
                (self.param.Nt + 1)
                * self.param.Nx : (self.param.Nt + 1)
                * self.param.Nx
                + self.param.Nu,
            ],
            self.param.Q,
            self.param.R,
        )

        # self.param.P = ricatti_recursion(
        #     -A[: self.param.Nx, : self.param.Nx],
        #     -A[
        #         : self.param.Nx,
        #         (self.param.Nt + 1)
        #         * self.param.Nx : (self.param.Nt + 1)
        #         * self.param.Nx
        #         + self.param.Nu,
        #     ],
        #     self.param.P,
        #     self.param.Q,
        #     self.param.R,
        # )

        M, q = self.mpc_cost(self.param.Q, self.param.R, self.param.P, xids)
        M = sparse.csr_matrix(M)
        A = sparse.csr_matrix(A)
        prob = osqp.OSQP()
        prob.setup(M, q, A, bmin, bmax, verbose=0)
        res = prob.solve()

        if res.info.status != "solved":
            raise ValueError("OSQP did not solve the problem!")
        u = res.x[
            (self.param.Nt + 1) * self.param.Nx : (self.param.Nt + 1) * self.param.Nx
            + self.param.Nu
        ]
        # return u 12x1
        return u

    def mpc_cost(self, Q, R, P, xids):
        # min x'Qx + (xi-xid)'Q(xi-xid) + u'Ru
        # xkp1 = Ak xk + Bk uk + bk, k = 0,1, ..., N-1
        # x0 = x_init,
        # umin < uk < umax, k = 0, 1, ..., N-1
        Nx = self.param.Nx
        Nu = self.param.Nu
        Nt = self.param.Nt

        M = np.zeros([(Nt + 1) * Nx + Nt * Nu, (Nt + 1) * Nx + Nt * Nu])
        q = np.zeros([(Nt + 1) * Nx + Nt * Nu, 1])
        d = np.zeros([12, 1])
        for k in range(Nt - 1):
            # iterate cost from k=1 to k=N-1
            k_ = k + 1
            C = np.eye(12)
            C[6:, :6] = -1 * adjoint(xids[k])
            M[k_ * Nx : (k_ + 1) * Nx, k_ * Nx : (k_ + 1) * Nx] = C.T @ Q @ C

            d[:] = 0
            d[6:] = xids[k]
            d = C.T @ Q @ d
            q[k_ * Nx : (k_ + 1) * Nx] = -d

        # terminal cost
        k_ = Nt
        C = np.eye(12)
        C[6:, :6] = -1 * adjoint(xids[Nt - 1])
        M[k_ * Nx : (k_ + 1) * Nx, k_ * Nx : (k_ + 1) * Nx] = C.T @ P @ C
        d = np.zeros([12, 1])
        d[6:] = xids[Nt - 1]
        d = C.T @ Q @ d
        q[k_ * Nx : (k_ + 1) * Nx] = -d

        Noff = Nx * (Nt + 1)
        for k in range(Nt):
            idx = Noff + k * Nu
            idx_1 = Noff + (k + 1) * Nu
            M[idx:idx_1, idx:idx_1] = R
        return [M, q]

    def mpc_constraint(self, xids, p0, xi0, X0, rs):
        # xids, desired twist list
        # xi0, current twist
        # p0, current state, [error ; twist]
        # rs, leverage of each footend, for swing leg rs = [0;0;0]

        # min x'Qx + u'Ru
        # xkp1 = Ak xk + Bk uk + bk, k = 0,1, ..., N-1
        # x0 = x_init,
        # umin < uk < umax, k = 0, 1, ..., N-1

        I = self.param.I
        dt = self.param.dt
        Nx = self.param.Nx
        Nu = self.param.Nu
        Nt = self.param.Nt
        Noff = Nx * (Nt + 1)
        # constraint Matrices
        A = np.zeros([Nx * (Nt + 1) + Nu * Nt, Nt * (Nx + Nu) + Nx])
        bmin = np.zeros([Nx * (Nt + 1) + Nu * Nt, 1])

        Ac = np.zeros([12, 12])
        Bc = np.zeros([12, 8])
        Gc = np.block(
            [[np.zeros([9, 1])], [X0[:3, :3].T @ np.array([[0], [0], [self.g]])]]
        )

        for k in range(Nt):
            xi_bar = I @ xi0
            G = np.zeros([6, 6])
            G[:3, :3] = skew(xi_bar[:3])
            G[:3, 3:] = skew(xi_bar[3:])
            G[3:, :3] = skew(xi_bar[3:])
            H = np.linalg.inv(I) @ (coadjoint(xi0) @ I + G)
            b = -np.linalg.inv(I) @ G @ xi0

            # 12x12
            Ac = np.block([[-adjoint(xids[k]), np.eye(6)], [np.zeros([6, 6]), H]])
            Ib_inv = np.linalg.inv(I[:3, :3])
            Bc = np.block(
                [
                    [np.zeros([6, 12])],
                    [
                        Ib_inv @ skew(rs[0]),
                        Ib_inv @ skew(rs[1]),
                        Ib_inv @ skew(rs[2]),
                        Ib_inv @ skew(rs[3]),
                    ],
                    [
                        np.eye(3) / self.m,
                        np.eye(3) / self.m,
                        np.eye(3) / self.m,
                        np.eye(3) / self.m,
                    ],
                ]
            )  # 12x12

            hc = np.block([[-xids[k]], [b]]) + Gc

            Ad = np.eye(Nx) + Ac * dt
            Bd = Bc * dt
            hd = hc * dt

            A[k * Nx : (k + 1) * Nx, k * Nx : (k + 1) * Nx] = -Ad
            A[k * Nx : (k + 1) * Nx, (k + 1) * Nx : (k + 2) * Nx] = np.eye(Nx)
            A[k * Nx : (k + 1) * Nx, (Noff + k * Nu) : (Noff + (k + 1) * Nu)] = -Bd
            A[
                Noff + k * Nu : Noff + (k + 1) * Nu, Noff + k * Nu : Noff + (k + 1) * Nu
            ] = np.eye(Nu)
            bmin[k * Nx : (k + 1) * Nx] = hd

        A[Nt * Nx : Noff, 0:Nx] = np.eye(Nx)  # for initial condition
        bmin[Nt * Nx : Noff] = p0
        bmax = bmin.copy()
        bmax[Noff:] = np.tile(self.param.umax, [Nt, 1])
        bmin[Noff:] = np.tile(self.param.umin, [Nt, 1])
        return [A, bmin, bmax]

    def simulation(self, q0, p0, w0, v0, rs0, dt, Nsim):
        # initial state q0(wxyz)
        X_refs = self.param.X_ref
        Xi_refs = self.param.xi_ref

        x_ = np.vstack(
            [q0.reshape(-1, 1), p0.reshape(-1, 1), w0.reshape(-1, 1), v0.reshape(-1, 1)]
        )
        R0 = quat2rotm(q0)
        X_se3_0 = np.block([[R0, p0], [np.array([0, 0, 0, 1])]])
        cp_world_ = [
            (X_se3_0 @ np.vstack([rs0[i].reshape(3, 1), 1]))[:3, 0].reshape(3, 1)
            for i in range(4)
        ]
        rs_ = rs0
        # print(cp_world_)
        lpfs = [LowPassFilter(10, 1 / self.param.dt, 1) for _ in range(6)]

        for i in range(Nsim - self.param.Nt):
            # print("k = ", i, end="\r")
            X_ref_rt = X_refs[i]
            xi_refs_rt = Xi_refs[i : (i + self.param.Nt)]

            # convert cuurent state to SE3
            q_ = x_[:4]
            p_ = x_[4:7]
            # print("p_: ", p_.reshape(3))
            X_SE3_ = np.block(
                [
                    [quat2rotm(q_), p_.reshape(3, 1)],
                    [0, 0, 0, 1],
                ]
            )

            rs_ = [
                (SE3inv(X_SE3_) @ np.vstack([cp_world_[i], 1]))[:3] for i in range(4)
            ]

            # print("rs\n", rs_[0].T)

            u = self.mpc(
                X_SE3_, X_ref_rt, x_[7:], xi_refs_rt, rs_
            )  # return 12x1 forces

            f_net = np.zeros([3, 1])
            tau_net = np.zeros([3, 1])
            for i in range(4):
                f_ = np.array([[u[3 * i]], [u[3 * i + 1]], [u[3 * i + 2]]])
                f_net += f_
                tau_net += skew(rs_[i]) @ f_
            u_net = np.vstack([tau_net, f_net])

            t_span = [0, dt]
            t_eval = np.arange(t_span[0], t_span[1] + dt / 40, dt / 40)
            sol = solve_ivp(
                SE3Dyn,
                t_span,
                x_.reshape(13),
                args=(u_net, self.param.I),
                t_eval=t_eval,
                method="RK45",
            )
            x_ = sol.y[:, -1].reshape(-1, 1)

            for i in range(6):
                x_[i + 7, 0] = lpfs[i].update(x_[i + 7, 0])

            # print("-- ode45 --")
            # print(u[2], u[5], u[8], u[11])
            # print(x_.reshape(13))

            qr_ = R.from_matrix(X_ref_rt[:3, :3]).as_quat()
            pr_ = X_ref_rt[:3, 3]
            x_ref_ = np.array([qr_[3], qr_[0], qr_[1], qr_[2], pr_[0], pr_[1], pr_[2]])
            x_ref_ = np.hstack([x_ref_, xi_refs_rt[0].reshape(6)])
            self.logger.appendRef(x_ref_)
            self.logger.appendState(x_)


if __name__ == "__main__":
    Ns = 12  # state dim.
    Nu = 12  # input dim.
    Nt = 12  # horizon
    dt = 0.025  # control frequency
    Tsim = 20
    Nsim = math.ceil(Tsim / dt)
    xmax = np.full((Ns, 1), np.inf)
    xmin = np.full((Ns, 1), -np.inf)
    umax = np.full((Nu, 1), 4000)
    umin = np.full((Nu, 1), -4000)
    param_ = param(Ns, Nu, Nt, dt, xmax, xmin, umax, umin)

    # Generate Reference trajectory
    p0_ref = np.array([[0], [0], [0]])
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
    param_.X_ref = X_refs  # in SE3 form
    param_.xi_ref = xi_refs  # twist 6x1

    # initialize mpc class
    mpc_ = MPC(param_)

    q0_sim = np.array([[1], [0], [0], [0]])
    p0_sim = np.array([[0], [0], [0]]) * 0
    w0_sim = np.array([[0], [0], [1]]) * 0
    v0_sim = np.array([[0.00], [0], [0]]) * 0
    dim_wb = 0.44
    dim_l = 0.6
    dim_w = 0.4
    dim_h = 0.15
    rs0_sim = [
        np.array([dim_wb / 2, dim_w / 2, -0.2]),
        np.array([dim_wb / 2, -dim_w / 2, -0.2]),
        np.array([-dim_wb / 2, -dim_w / 2, -0.2]),
        np.array([-dim_wb / 2, dim_w / 2, -0.2]),
    ]

    start = time.time()
    mpc_.simulation(q0_sim, p0_sim, w0_sim, v0_sim, rs0_sim, dt=dt, Nsim=Nsim)
    print("elapsed, ", time.time() - start)
    mpc_.logger.plotTrajectory(Nskip=100)
