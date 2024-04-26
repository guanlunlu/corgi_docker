import numpy as np
import scipy as sp
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter_zi, lfilter


def skew(vx):
    x = vx.reshape(3)
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def adjoint(x):
    # adjoint Matrix in Lie Algebra (se3)
    # twist: x = [w; v]
    skw = skew(x[:3])
    skv = skew(x[3:6])
    adx = np.block([[skw, np.zeros((3, 3))], [skv, skw]])
    return adx


def Adjoint(T):
    # Adjoint Matrix in Lie Group (SE3)
    R = T[:3, :3]
    p = T[:3, 3]
    Adx = np.block([[R, np.zeros((3, 3))], [skew(p) @ R, R]])
    pass


def coadjoint(x):
    # twist: x = [w; v]
    return np.transpose(adjoint(x))


def quat2rotm(quaternion_wxyz):
    # for scipy, quat: xyzw
    q = quaternion_wxyz.reshape(4)
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()


def SE3inv(X):
    R = X[:3, :3]
    p = X[:3, 3].reshape(3, 1)
    return np.block([[R.T, -R.T @ p], [np.array([0, 0, 0, 1])]])


def ricatti_recursion(A, B, P, Q, R):
    # iterate backward
    P_k_1 = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    return P_k_1


def SE3Dyn(t, x, u, I):
    q_wxyz = np.array([[x[0]], [x[1]], [x[2]], [x[3]]])
    p = np.array([[x[4]], [x[5]], [x[6]]])
    w = np.array([[x[7]], [x[8]], [x[9]]])
    v = np.array([[x[10]], [x[11]], [x[12]]])
    Omega = np.array(
        [
            [0, -w[0, 0], -w[1, 0], -w[2, 0]],
            [w[0, 0], 0, w[2, 0], -w[1, 0]],
            [w[1, 0], -w[2, 0], 0, w[0, 0]],
            [w[2, 0], w[1, 0], -w[0, 0], 0],
        ]
    )
    dQuat = 0.5 * Omega @ q_wxyz

    R = quat2rotm(q_wxyz)
    # G_ = R.T @ np.array([0, 0, -9.81]).reshape(3, 1)
    G = I @ np.vstack([np.zeros([3, 1]), R.T @ np.array([0, 0, -9.81]).reshape(3, 1)])
    # print(np.vstack([np.zeros([3, 1]), R.T @ np.array([0, 0, -9.81]).reshape(3, 1)]))

    dx = np.concatenate([dQuat, R @ v])
    d2x = np.linalg.inv(I) @ (
        coadjoint(np.concatenate([w, v])) @ I @ np.concatenate([w, v]) + u.reshape(6, 1)
    ) + np.vstack([np.zeros([3, 1]), R.T @ np.array([0, 0, -9.81]).reshape(3, 1)])
    dxdt = np.hstack([dx.T, d2x.T]).ravel()
    return dxdt


class LowPassFilter:
    def __init__(self, cutoff, fs, order=5):
        self.cutoff = cutoff
        self.fs = fs
        self.order = order
        self.b, self.a = self._butter_lowpass()
        self.zi = lfilter_zi(self.b, self.a) * 0  # Initialize zi with zero

    def _butter_lowpass(self):
        nyq = 0.5 * self.fs
        normal_cutoff = self.cutoff / nyq
        b, a = butter(self.order, normal_cutoff, btype="low", analog=False)
        return b, a

    def apply_filter(self, data_stream):
        # Applies the filter to a sequence of data points (stream)
        filtered_stream, self.zi = lfilter(self.b, self.a, data_stream, zi=self.zi)
        return filtered_stream

    def update(self, sample):
        # Update the filter with a new sample, useful for real-time applications
        filtered_sample, self.zi = lfilter(self.b, self.a, [sample], zi=self.zi)
        return filtered_sample[0]


if __name__ == "__main__":
    rotv = np.array([0, 0, 1])
    rotm = sp.linalg.expm(skew(rotv))
    print(rotm)
    r_ = R.from_matrix(rotm)
    print(r_.as_euler("zyx"))
