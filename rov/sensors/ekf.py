import math
import numpy as np
import time

class EKF:
    """
    Error-State Extended Kalman Filter (ESKF) for 6-DOF IMU (Gyro + Accel) + optional Yaw (Vision).
    State vector (nominal):
      q: Quaternion [w, x, y, z] (Attitude)
      b_g: Gyro bias [x, y, z] (rad/s)
    Error state vector:
      d_theta: Attitude error [x, y, z]
      d_bg: Gyro bias error [x, y, z]
    """
    def __init__(self, dt=0.01):
        self.dt = dt
        
        # Nominal State
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z
        self.bg = np.array([0.0, 0.0, 0.0])     # rad/s
        
        # Covariance Matrix (6x6)
        self.P = np.eye(6) * 0.01
        
        # Process Noise Covariance (Q)
        self.Q = np.eye(6)
        self.Q[0:3, 0:3] *= (0.01)**2  # Gyro noise
        self.Q[3:6, 3:6] *= (0.001)**2 # Gyro bias random walk
        
        # Measurement Noise Covariance (Accel)
        self.R_acc = np.eye(3) * (0.1)**2 # Accel noise
        
        # Measurement Noise Covariance (Yaw)
        self.R_yaw = np.eye(1) * (0.05)**2 # Yaw noise

    def predict(self, gyro, dt):
        """
        gyro: [gx, gy, gz] in rad/s
        dt: time step in seconds
        """
        if dt <= 0: return

        # Correct gyro with bias
        w = np.array(gyro) - self.bg
        
        # Update nominal quaternion (Runge-Kutta 1st order)
        w_norm = np.linalg.norm(w)
        if w_norm > 1e-6:
            dq = np.array([
                math.cos(w_norm * dt / 2),
                *( (w / w_norm) * math.sin(w_norm * dt / 2) )
            ])
            self.q = self.quat_mult(self.q, dq)
            self.q /= np.linalg.norm(self.q)
            
        # Error state transition matrix (Phi)
        Phi = np.eye(6)
        wx = self.skew_symmetric(w)
        Phi[0:3, 0:3] = np.eye(3) - wx * dt
        Phi[0:3, 3:6] = -np.eye(3) * dt
        
        # Update Covariance
        self.P = Phi @ self.P @ Phi.T + self.Q * dt
        
    def update_accel(self, acc):
        """
        acc: [ax, ay, az] in g's
        """
        acc = np.array(acc)
        acc_norm = np.linalg.norm(acc)
        if acc_norm < 0.5 or acc_norm > 1.5:
            return # Ignore if too much linear acceleration
            
        acc_normed = acc / acc_norm
        
        # Expected gravity vector in body frame
        g_body = np.array([
            2 * (self.q[1]*self.q[3] - self.q[0]*self.q[2]),
            2 * (self.q[2]*self.q[3] + self.q[0]*self.q[1]),
            self.q[0]**2 - self.q[1]**2 - self.q[2]**2 + self.q[3]**2
        ])
        
        # Residual
        res = acc_normed - g_body
        
        # Jacobian of measurement w.r.t error state
        H = np.zeros((3, 6))
        H[0:3, 0:3] = self.skew_symmetric(g_body)
        
        # Kalman Gain
        S = H @ self.P @ H.T + self.R_acc
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Compute error state
        dx = K @ res
        
        # Inject error into nominal state
        self.inject_error(dx)
        
        # Update Covariance
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_yaw(self, yaw_meas):
        """
        yaw_meas: Absolute yaw measurement in radians
        """
        r, p, y = self.get_euler()
        
        # Residual (shortest angular distance)
        res = np.array([(yaw_meas - y + math.pi) % (2*math.pi) - math.pi])
        
        R = self.quat_to_rot(self.q)
        H = np.zeros((1, 6))
        H[0, 0:3] = R[2, :] # Z-row of rotation matrix
        
        S = H @ self.P @ H.T + self.R_yaw
        K = self.P @ H.T @ np.linalg.inv(S)
        
        dx = K @ res
        self.inject_error(dx)
        self.P = (np.eye(6) - K @ H) @ self.P

    def inject_error(self, dx):
        d_theta = dx[0:3]
        d_bg = dx[3:6]
        
        self.bg += d_bg
        
        dq = np.array([1.0, d_theta[0]/2, d_theta[1]/2, d_theta[2]/2])
        self.q = self.quat_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)

    def get_euler(self):
        """Returns Roll, Pitch, Yaw in radians (Z-Y-X convention)"""
        w, x, y, z = self.q
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    @staticmethod
    def skew_symmetric(v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    @staticmethod
    def quat_mult(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
        
    @staticmethod
    def quat_to_rot(q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
        ])
