"""
MPC Solver wrapper class.
참고 레포 mpc_controller.py의 run_step 핵심 로직을 추출하여 ROS 의존성 제거.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acados_settings import acados_settings


class MPCSolver:
    """
    Frenet dynamic bicycle MPC solver wrapper.
    state x = [s, n, alpha, v, D, delta]
    input u = [derD, derDelta]
    """

    def __init__(self, coeffs, knots, Tf=2.0, N=20, target_speed=8.0,
                 global_path_length=200.0, degree=3,
                 json_file="acados_ocp.json"):
        """
        Args:
            coeffs, knots : kappa(s) B-spline (from compute_kappa_spline_for_acados)
            Tf            : prediction horizon [s]
            N             : horizon steps
            target_speed  : reference speed [m/s]
            global_path_length : 경로 전체 길이 (s_total)
        """
        self.Tf = Tf
        self.N  = N
        self.dt = Tf / N
        self.target_speed = target_speed
        self.global_path_length = global_path_length

        print("[MPCSolver] building solver...")
        self.constraint, self.model, self.solver = acados_settings(
            Tf, N, coeffs, knots, degree=degree, json_file=json_file)
        print("[MPCSolver] solver built")

    def solve(self, x0):
        """
        Args:
            x0 : np.array of shape (6,) = [s, n, alpha, v, D, delta]
        Returns:
            target_D, target_delta, status
        """
        s, n, alpha, v, D, delta = x0

        # --- distance to stop (감속 여유) ---
        distance2stop = 0.5 * v

        # --- yref for stages 1..N-1 ---
        for i in range(1, self.N):
            s_target = s + self.target_speed * self.dt * (i + 1)
            if s_target > self.global_path_length:
                s_target = self.global_path_length - distance2stop
            yref = np.array([s_target, 0, 0, self.target_speed, 0, 0, 0, 0])
            self.solver.set(i, "yref", yref)

            self.solver.constraints_set(i, "lh", np.array([
                self.constraint.along_min,
                self.constraint.alat_min,
                self.model.n_min,
                self.model.v_min,
                self.model.throttle_min,
                self.model.delta_min,
            ]))
            self.solver.constraints_set(i, "uh", np.array([
                self.constraint.along_max,
                self.constraint.alat_max,
                self.model.n_max,
                self.model.v_max,
                self.model.throttle_max,
                self.model.delta_max,
            ]))

        # --- terminal yref ---
        s_target_N = s + self.target_speed * self.Tf
        if s_target_N > self.global_path_length:
            s_target_N = self.global_path_length - distance2stop
        yref_N = np.array([s_target_N, 0, 0, self.target_speed, 0, 0])
        self.solver.set(self.N, "yref", yref_N)

        # --- initial condition (force x0) ---
        x0_arr = np.array(x0)
        self.solver.constraints_set(0, "lbx", x0_arr)
        self.solver.constraints_set(0, "ubx", x0_arr)

        # --- solve ---
        status = self.solver.solve()

        # --- extract control from x at stage 1 ---
        # 참고 레포: x0 = self.acados_solver.get(1, "x"); target_D=x0[4]; target_delta=x0[5]
        x1 = self.solver.get(1, "x")
        target_D     = float(x1[4])
        target_delta = float(x1[5])

        return target_D, target_delta, status

    def get_predicted_trajectory(self):
        """디버깅용: horizon 내 예측 trajectory를 (N+1, nx) 배열로 반환."""
        traj = np.zeros((self.N + 1, 6))
        for i in range(self.N + 1):
            traj[i] = self.solver.get(i, "x")
        return traj