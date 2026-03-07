import numpy as np

def ECE569_NearZero(z):
    return abs(z) < 1e-6

def ECE569_Normalize(V):
    return V / np.linalg.norm(V)


def ECE569_RotInv(R):
    return np.array(R).T

def ECE569_VecToso3(omg):
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])

def ECE569_so3ToVec(so3mat):
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])

def ECE569_AxisAng3(expc3):
    return (ECE569_Normalize(expc3), np.linalg.norm(expc3))

def ECE569_MatrixExp3(so3mat):
    omgtheta = ECE569_so3ToVec(so3mat)
    if ECE569_NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = ECE569_AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)

def ECE569_MatrixLog3(R):
    acosinput = (np.trace(R) - 1) / 2.0
    if acosinput >= 1:
        return np.zeros((3, 3))
    elif acosinput <= -1:
        if not ECE569_NearZero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not ECE569_NearZero(1 + R[1][1]):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return ECE569_VecToso3(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)

def ECE569_TransToRp(T):
    T = np.array(T)
    return T[0:3, 0:3], T[0:3, 3]

def ECE569_TransInv(T):
    R, p = ECE569_TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -Rt @ p], [[0, 0, 0, 1]]]

def ECE569_VecTose3(V):
    wx, wy, wz, vx, vy, vz = V
    w_hat = np.array([[0, -wz, wy],
                      [wz, 0, -wx],
                      [-wy, wx, 0]])
    v = np.array([[vx], [vy], [vz]])
    return np.block([[w_hat, v],
                     [np.zeros((1, 3)), np.zeros((1, 1))]])

def ECE569_se3ToVec(se3mat):
    w_hat = se3mat[0:3, 0:3]
    v = se3mat[0:3, 3]
    w = ECE569_so3ToVec(w_hat)
    return np.array([w[0], w[1], w[2], v[0], v[1], v[2]])

def ECE569_Adjoint(T):
    R, p = ECE569_TransToRp(T)
    p_hat = ECE569_VecToso3(p)
    return np.block([[R, np.zeros((3, 3))], [p_hat @ R, R]])

def ECE569_MatrixExp6(se3mat):
    se3mat = np.array(se3mat)
    omega_mat = se3mat[0:3, 0:3]
    v = se3mat[0:3, 3]
    omgtheta = ECE569_so3ToVec(omega_mat)
    if ECE569_NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3), v.reshape(3, 1)], [[0, 0, 0, 1]]]
    theta = np.linalg.norm(omgtheta)
    omega_hat = omega_mat / theta
    R = ECE569_MatrixExp3(omega_mat)
    I3 = np.eye(3)
    G = I3 * theta \
        + (1 - np.cos(theta)) * omega_hat \
        + (theta - np.sin(theta)) * (omega_hat @ omega_hat)
    p = (G @ (v / theta)).reshape(3, 1)
    return np.block([[R, p], [np.zeros((1, 3)), np.array([[1]])]])

def ECE569_MatrixLog6(T):
    R, p = ECE569_TransToRp(T)
    omgmat = ECE569_MatrixLog3(R)
    if np.allclose(omgmat, np.zeros((3, 3))):
        if np.linalg.norm(p) < 1e-6:
            return np.zeros((4, 4))
        return np.block([[np.zeros((3, 3)), p.reshape(3, 1)],
                         [np.zeros((1, 3)), np.zeros((1, 1))]])
    omega_theta = ECE569_so3ToVec(omgmat)
    theta = np.linalg.norm(omega_theta)
    omega_hat = omgmat / theta
    I3 = np.eye(3)
    G_inv = (1.0 / theta) * I3 \
        - 0.5 * omega_hat \
        + (1.0 / theta - 0.5 / np.tan(theta / 2.0)) * (omega_hat @ omega_hat)
    v = G_inv @ p
    return np.r_[np.c_[omgmat, (v * theta).reshape(3, 1)], [np.zeros(4)]]

'''
*** CHAPTER 4: FORWARD KINEMATICS ***
'''

def ECE569_FKinBody(M, Blist, thetalist):
    T = M.copy()
    for i in range(len(thetalist)):
        Bi = Blist[:, i]
        se3 = ECE569_VecTose3(Bi * thetalist[i])
        T = T @ ECE569_MatrixExp6(se3)
    return T

def ECE569_FKinSpace(M, Slist, thetalist):
    T = np.array(M)
    for i in range(len(thetalist) - 1, -1, -1):
        Si = Slist[:, i]
        se3 = ECE569_VecTose3(Si * thetalist[i])
        T = ECE569_MatrixExp6(se3) @ T
    return T

'''
*** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***
'''

def ECE569_JacobianBody(Blist, thetalist):
    Jb = np.array(Blist).copy().astype(float)
    T = np.eye(4)
    for i in range(len(thetalist) - 1, 0, -1):
        Bi = Blist[:, i]
        T = T @ ECE569_MatrixExp6(ECE569_VecTose3(-Bi * thetalist[i]))
        Jb[:, i - 1] = ECE569_Adjoint(T) @ Blist[:, i - 1]
    return Jb

'''
*** CHAPTER 6: INVERSE KINEMATICS ***
'''

def ECE569_IKinBody(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 1000
    Tsb = ECE569_FKinBody(M, Blist, thetalist)
    Tbd = ECE569_TransInv(Tsb) @ T
    Vb = ECE569_se3ToVec(ECE569_MatrixLog6(Tbd))
    err = np.linalg.norm(Vb[:3]) > eomg or np.linalg.norm(Vb[3:]) > ev
    while err and i < maxiterations:
        Jb = ECE569_JacobianBody(Blist, thetalist)
        thetalist = thetalist + np.linalg.pinv(Jb) @ Vb
        i += 1
        Tsb = ECE569_FKinBody(M, Blist, thetalist)
        Tbd = ECE569_TransInv(Tsb) @ T
        Vb = ECE569_se3ToVec(ECE569_MatrixLog6(Tbd))
        err = np.linalg.norm(Vb[:3]) > eomg or np.linalg.norm(Vb[3:]) > ev
    return (thetalist, not err)


# =====================================================================
# UR3e constants
# =====================================================================

L1 = 0.2435
L2 = 0.2132
W1 = 0.1311
W2 = 0.0921
H1 = 0.1519
H2 = 0.0854

M_HOME = np.array([[-1, 0, 0, L1 + L2],
                    [ 0, 0, 1, W1 + W2],
                    [ 0, 1, 0, H1 - H2],
                    [ 0, 0, 0, 1]])

_S1 = np.array([0, 0, 1, 0, 0, 0])
_S2 = np.array([0, 1, 0, -H1, 0, 0])
_S3 = np.array([0, 1, 0, -H1, 0, L1])
_S4 = np.array([0, 1, 0, -H1, 0, L1 + L2])
_S5 = np.array([0, 0, -1, -W1, L1 + L2, 0])
_S6 = np.array([0, 1, 0, H2 - H1, 0, L1 + L2])
S_AXES = np.array([_S1, _S2, _S3, _S4, _S5, _S6]).T

_AdM_inv = np.linalg.inv(ECE569_Adjoint(M_HOME))
B_AXES = np.array([_AdM_inv @ s for s in [_S1, _S2, _S3, _S4, _S5, _S6]]).T

THETA0 = np.array([-1.6800, -1.4018, -1.8127, -2.9937, -0.8857, -0.0696])


# =====================================================================
# Pipeline functions
# =====================================================================

def solve_trajectory(x_m, y_m, pen, eomg=1e-3, ev=1e-3):
    """Solve IK for a sequence of 2D drawing-plane displacements.

    Args:
        x_m, y_m: arrays of EE displacement in meters (relative to home pose)
        pen: array of 0/1 pen states
        eomg, ev: IK tolerances

    Returns:
        thetaAll: (6, N) joint angles, or None on early failure
        min_det: minimum |det(Jb)| across trajectory (singularity metric)
        success: True if every IK solve converged
    """
    N = len(x_m)
    T0 = ECE569_FKinSpace(M_HOME, S_AXES, THETA0)

    # build desired EE poses
    Tsd = np.zeros((4, 4, N))
    for i in range(N):
        Td = np.eye(4)
        Td[0:3, 3] = np.array([x_m[i], y_m[i], 0.0])
        Tsd[:, :, i] = T0 @ Td

    # IK with continuity seeding
    thetaAll = np.zeros((6, N))
    thetaSol, ok = ECE569_IKinBody(B_AXES, M_HOME, Tsd[:, :, 0], THETA0, eomg, ev)
    if not ok:
        return None, None, False
    thetaAll[:, 0] = thetaSol

    for i in range(1, N):
        thetaSol, ok = ECE569_IKinBody(B_AXES, M_HOME, Tsd[:, :, i],
                                        thetaAll[:, i - 1], eomg, ev)
        if not ok:
            return thetaAll, None, False
        thetaAll[:, i] = thetaSol

    # singularity check
    min_det = float('inf')
    for i in range(N):
        d = abs(np.linalg.det(ECE569_JacobianBody(B_AXES, thetaAll[:, i])))
        if d < min_det:
            min_det = d

    return thetaAll, min_det, True


def export_csv(filepath, thetaAll, pen, dt=0.002):
    """Save joint trajectory as CSV compatible with csv_controller.

    Format per row: t, q1, q2, q3, q4, q5, q6, pen
    """
    N = thetaAll.shape[1]
    t = np.arange(N) * dt
    data = np.column_stack([t, thetaAll.T, pen])
    np.savetxt(filepath, data, delimiter=',')
