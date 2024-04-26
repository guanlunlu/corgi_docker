import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as R
import matplotlib as mpl

# mpl.rcParams["figure.dpi"] = 200


class logger:
    def __init__(self) -> None:
        self.xs = np.empty([1, 13])
        self.refs = np.empty([1, 13])
        self.xs_init = False
        self.refs_init = False

    def appendState(self, x_):
        # x_ = qw, qx, qy, qz, px, py, pz, wx, wy, wz, vx, vy, vz
        if self.xs_init:
            self.xs = np.vstack([self.xs, x_.reshape(1, 13)])
        else:
            self.xs = x_.reshape(1, 13)
            self.xs_init = True

    def appendRef(self, x_):
        if self.refs_init:
            self.refs = np.vstack([self.refs, x_.reshape(1, 13)])
        else:
            self.refs = x_.reshape(1, 13)
            self.refs_init = True

    def plotTrajectory(self, Nskip=40, axislen=0.1):

        qw_ = self.xs[:, 0]
        qx_ = self.xs[:, 1]
        qy_ = self.xs[:, 2]
        qz_ = self.xs[:, 3]
        px_ = self.xs[:, 4]
        py_ = self.xs[:, 5]
        pz_ = self.xs[:, 6]
        wx_ = self.xs[:, 7]
        wy_ = self.xs[:, 8]
        wz_ = self.xs[:, 9]
        vx_ = self.xs[:, 10]
        vy_ = self.xs[:, 11]
        vz_ = self.xs[:, 12]

        rqw_ = self.refs[:, 0]
        rqx_ = self.refs[:, 1]
        rqy_ = self.refs[:, 2]
        rqz_ = self.refs[:, 3]
        rpx_ = self.refs[:, 4]
        rpy_ = self.refs[:, 5]
        rpz_ = self.refs[:, 6]
        rwx_ = self.refs[:, 7]
        rwy_ = self.refs[:, 8]
        rwz_ = self.refs[:, 9]
        rvx_ = self.refs[:, 10]
        rvy_ = self.refs[:, 11]
        rvz_ = self.refs[:, 12]

        """ fig = plt.figure(1)
        # ax = plt.axes(projection="3d")
        ax = fig.add_subplot(111, projection="3d")
        ax.plot3D(px_[1:], py_[1:], pz_[1:], label="state", color="blue", alpha=0.7)
        ax.plot3D(
            rpx_[1:], rpy_[1:], rpz_[1:], "--", label="refs", color="red", alpha=0.7
        )

        i = 0
        ps_ = np.empty([1, 3])
        rps_ = np.empty([1, 3])
        while i < self.xs.shape[0]:
            R_ = R.from_quat([qx_[i], qy_[i], qz_[i], qw_[i]]).as_matrix()
            p_ = np.array([px_[i], py_[i], pz_[i]])
            x_ = p_ + R_[:, 0] * axislen
            y_ = p_ + R_[:, 1] * axislen
            z_ = p_ + R_[:, 2] * axislen
            ax.plot3D([px_[i], x_[0]], [py_[i], x_[1]], [pz_[i], x_[2]], color="r")
            ax.plot3D([px_[i], y_[0]], [py_[i], y_[1]], [pz_[i], y_[2]], color="g")
            ax.plot3D([px_[i], z_[0]], [py_[i], z_[1]], [pz_[i], z_[2]], color="b")
            ps_ = np.vstack([ps_, p_])
            # ax.scatter(px_[i], py_[i], pz_[i], c=[i], cmap="Blues")

            rR_ = R.from_quat([rqx_[i], rqy_[i], rqz_[i], rqw_[i]]).as_matrix()
            rp_ = np.array([rpx_[i], rpy_[i], rpz_[i]])
            rx_ = rp_ + rR_[:, 0] * axislen
            ry_ = rp_ + rR_[:, 1] * axislen
            rz_ = rp_ + rR_[:, 2] * axislen
            ax.plot3D(
                [rpx_[i], rx_[0]], [rpy_[i], rx_[1]], [rpz_[i], rx_[2]], color="r"
            )
            ax.plot3D(
                [rpx_[i], ry_[0]], [rpy_[i], ry_[1]], [rpz_[i], ry_[2]], color="g"
            )
            ax.plot3D(
                [rpx_[i], rz_[0]], [rpy_[i], rz_[1]], [rpz_[i], rz_[2]], color="b"
            )
            rps_ = np.vstack([rps_, rp_])

            i += Nskip """

        # ax.scatter(
        #     ps_[1:, 0],
        #     ps_[1:, 1],
        #     ps_[1:, 2],
        #     s=10,
        #     # c=np.linspace(0, 1, ps_.shape[0] - 1),
        #     cmap="rainbow",
        #     alpha=1,
        # )

        # ax.scatter(
        #     rps_[:, 0],
        #     rps_[:, 1],
        #     rps_[:, 2],
        #     s=10,
        #     c=np.linspace(0, 1, ps_.shape[0]),
        #     cmap="rainbow",
        #     alpha=1,
        # )

        # ax.set_xlabel("X")
        # ax.set_ylabel("Y")
        # ax.set_zlabel("Z")
        # # ax.set_box_aspect([1, 1, 1])
        # ax.set_xlim3d([-0.5, 2])
        # ax.set_ylim3d([-0.5, 2])
        # ax.set_zlim3d([0, 2.5])
        # plt.legend()

        fig = plt.figure(2, figsize=(15, 10))
        t = np.linspace(0, px_.shape[0] * 0.025, px_.shape[0])
        plt.subplot(321)
        plt.plot(t, px_[0:], label="state")
        plt.plot(t, rpx_[0:], label="reference")
        plt.legend()

        plt.subplot(323)
        plt.plot(t, py_[0:], label="state")
        plt.plot(t, rpy_[0:], label="reference")
        plt.legend()

        plt.subplot(325)
        plt.plot(t, pz_[0:], label="state")
        plt.plot(t, rpz_[0:], label="reference")
        plt.legend()

        plt.subplot(322)
        plt.plot(t, vx_[0:], label="state")
        plt.plot(t, rvx_[0:], label="reference")
        plt.legend()

        plt.subplot(324)
        plt.plot(t, vy_[0:], label="state")
        plt.plot(t, rvy_[0:], label="reference")
        plt.legend()

        plt.subplot(326)
        plt.plot(t, vz_[0:], label="state")
        plt.plot(t, rvz_[0:], label="reference")
        plt.legend()

        plt.show()
