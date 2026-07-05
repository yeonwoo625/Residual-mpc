"""
MPC Solver with l4acados residual learning integration.
- Wraps acados OCP with l4acados ResidualLearningMPC
- Embeds trained PyTorch residual model
- Handles per-episode kappa spline update
"""
import os
import sys
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from acados_settings import acados_settings
from residual_model_wrapper import load_normalized_model
from kappa_feature_selector import KappaAwareFeatureSelector

# l4acados imports
from l4acados.controllers import ResidualLearningMPC
from l4acados.models import PyTorchResidualModel


class FeedforwardResidualModel(PyTorchResidualModel):
    """잔차 '값'은 그대로 쓰되 Jacobian을 0으로 반환 -> MPC가 잔차를
    조향으로 조절 가능한 항이 아니라 '고정 외란(feedforward)'으로 취급.
    이러면 잔차↔조향 되먹임 진동 루프가 구조적으로 사라진다 (RESIDUAL_FF=1).
    잔차 값은 매 SQP 반복마다 현재 예측 궤적에서 재평가되므로 보정은 유지된다.
    """
    def jacobian(self, y):
        rdim = self.current_prediction.shape[1]   # residual_dim (=4)
        N = self.current_prediction.shape[0]      # horizon
        sdim = np.asarray(y).shape[1]             # state+input dim
        self.current_prediction_dy = np.zeros((rdim, N, sdim))
        return self.current_prediction_dy


# Residual model output (4-dim: Δs, Δn, Δalpha, Δv)
# acados state (6-dim: s, n, alpha, v, D, delta)
# B matrix maps 4-dim residual to 6-dim state correction.
B_MATRIX = np.array([
    [0, 0, 0, 0],   # s     ← Δs 제거
    [0, 1, 0, 0],   # n     ← Δn 유지
    [0, 0, 1, 0],   # alpha ← Δalpha 유지
    [0, 0, 0, 0],   # v     ← Δv 제거 (lateral-only; Δv가 제일 크고 거칠어 발산 의심)
    [0, 0, 0, 0],
    [0, 0, 0, 0],
], dtype=np.float64)


class MPCSolverResidual:
    """
    Frenet dynamic bicycle MPC with learned residual.
    
    Per-episode usage:
        solver = MPCSolverResidual(coeffs, knots, model_path, ...)
        solver.update_path(dense_s, kappa_values)   # before each episode
        target_D, target_delta, status = solver.solve(x0)
    """
    
    def __init__(self,
                 coeffs, knots,
                 model_path,
                 Tf=2.0, N=20,
                 target_speed=8.0,
                 global_path_length=200.0,
                 degree=3,
                 device='cpu',
                 json_file_ocp="residual_lbmpc_ocp.json",
                 json_file_sim="residual_lbmpc_sim.json"):
        
        self.Tf = Tf
        self.N = N
        self.dt = Tf / N
        self.target_speed = target_speed
        self.global_path_length = global_path_length
        
        # ----- Build acados OCP (we don't directly use the resulting solver) -----
        print("[MPCSolverResidual] building acados OCP for residual learning...")
        self._raw_constraint, self._raw_model, _raw_solver = acados_settings(
            Tf, N, coeffs, knots, degree=degree,
            json_file="acados_ocp_for_residual.json",
        )
        # We need the OCP object itself; acados_settings builds and returns the solver,
        # but we need to expose the ocp for l4acados. So we regenerate ocp here.
        # Workaround: re-generate ocp manually.
        from acados_template import AcadosOcp, AcadosModel
        from bicycle_model import bicycle_model
        import scipy.linalg
        
        ocp = AcadosOcp()
        model, constraint = bicycle_model(self.dt, coeffs, knots, degree)
        
        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.f_impl_expr
        model_ac.f_expl_expr = model.f_expl_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.z = model.z
        model_ac.p = model.p
        model_ac.name = model.name
        ocp.model = model_ac
        model_ac.con_h_expr = constraint.expr
        
        nx, nu = 6, 2
        ny, ny_e = nx + nu, nx
        nh, nsh = constraint.expr.shape[0], 3
        
        ocp.dims.N = N
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        unscale = N / Tf
        
        Q = np.diag([1e-5, 1e-4, 1e-5, 1e-3, 0.0, 0.0])
        R = np.eye(nu)
        R[0, 0] = 2e-4
        R[1, 1] = 2e-4
        Qe = np.diag([5e-5, 1e-4, 1e-5, 0.0, 0.0, 0.0])
        
        ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Qe / unscale
        
        Vx = np.zeros((ny, nx)); Vx[:nx, :nx] = np.eye(nx)
        Vu = np.zeros((ny, nu)); Vu[6, 0] = 1.0; Vu[7, 1] = 1.0
        Vx_e = np.zeros((ny_e, nx)); Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx; ocp.cost.Vu = Vu; ocp.cost.Vx_e = Vx_e
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)
        
        ocp.constraints.lbu = np.array([model.dthrottle_min, model.ddelta_min])
        ocp.constraints.ubu = np.array([model.dthrottle_max, model.ddelta_max])
        ocp.constraints.idxbu = np.array([0, 1])
        
        ocp.constraints.lh = np.array([
            constraint.along_min, constraint.alat_min,
            model.n_min, model.v_min,
            model.throttle_min, model.delta_min,
        ])
        ocp.constraints.uh = np.array([
            constraint.along_max, constraint.alat_max,
            model.n_max, model.v_max,
            model.throttle_max, model.delta_max,
        ])
        
        slack_L1 = np.array([1e-6, 1e-6, 1e1])
        slack_L2 = np.array([1e-6, 1e-6, 1e1])
        ocp.cost.zl = slack_L1; ocp.cost.zu = slack_L1
        ocp.cost.Zl = slack_L2; ocp.cost.Zu = slack_L2
        ocp.constraints.lsh = np.zeros(nsh)
        ocp.constraints.ush = np.zeros(nsh)
        ocp.constraints.idxsh = np.array([0, 1, 2])
        
        ocp.constraints.x0 = model.x0
        
        ocp.solver_options.tf = Tf
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.levenberg_marquardt = 1e-3
        
        self.ocp = ocp
        self.constraint = constraint
        self.model = model
        
        # ----- Build PyTorch residual model + selector -----
        self.device = device
        self.torch_model = load_normalized_model(model_path, device=device)
        self.feature_selector = KappaAwareFeatureSelector()
        # spline은 update_path()로 나중에 갱신

        # 조건화 모델(8차원 입력)인데 MASS/COG_X를 안 주면 selector가 6차원을 만들어
        # 첫 solve에서 차원 불일치로 죽는다. 여기서 미리 명확히 알려준다.
        model_in = int(self.torch_model.X_mean.shape[0])
        sel_cond = bool(getattr(self.feature_selector, "cond", False))
        if model_in > 6 and not sel_cond:
            raise SystemExit(
                f"[MPCSolverResidual] CONDITIONED model expects {model_in}-dim input "
                f"but MASS/COG_X env not set (selector produces 6-dim). "
                f"Set e.g. MASS=48000 COG_X=4.330 before running the server.")
        if model_in == 6 and sel_cond:
            print("[MPCSolverResidual] WARNING: MASS/COG_X set but model is "
                  "unconditioned (6-dim); ignoring context.", flush=True)

        # RESIDUAL_FF=1 -> 피드포워드(Jacobian=0) 잔차: 조향 되먹임 진동 차단.
        _FF = os.environ.get("RESIDUAL_FF", "0") == "1"
        _RM = FeedforwardResidualModel if _FF else PyTorchResidualModel
        print(f"[MPCSolverResidual] residual mode = "
              f"{'FEEDFORWARD (jac=0)' if _FF else 'full (jac)'}", flush=True)
        self.residual_model = _RM(
            model=self.torch_model,
            feature_selector=self.feature_selector,
            use_jacfwd=True,
        )
        
        # ----- Build l4acados ResidualLearningMPC -----
        print("[MPCSolverResidual] building ResidualLearningMPC...")
        self.controller = ResidualLearningMPC(
            ocp=ocp,
            B=B_MATRIX,
            residual_model=self.residual_model,
            build_c_code=True,
            use_cython=True,
            path_json_ocp=json_file_ocp,
            path_json_sim=json_file_sim,
        )
        print("[MPCSolverResidual] solver ready")

        # closed-loop residual diagnostic (RESIDUAL_DEBUG=1): is the oscillation
        # from VALUE blowup (clamp hit often -> extrapolation -> need data) or a
        # rough JACOBIAN (values calm but control jitters -> need jac_reg)?
        self._dbg = os.environ.get("RESIDUAL_DEBUG", "0") == "1"
        self._dbg_n = 0
        self._dbg_hits = np.zeros(4, dtype=int)
        self._dbg_maxabs = np.zeros(4)

    def update_path(self, dense_s, kappa_values, degree=3):
        """매 episode 시작 시 호출. selector의 kappa_spline 갱신."""
        self.feature_selector.update_kappa_spline(dense_s, kappa_values, degree=degree)
    
    def solve(self, x0):
        """
        Args:
            x0: np.array shape (6,) = [s, n, alpha, v, D, delta]
        Returns:
            target_D, target_delta, status
        """
        s, n, alpha, v, D, delta = x0

        if self._dbg:
            y0 = torch.tensor([[s, n, alpha, v, D, delta, 0.0, 0.0]],
                              dtype=torch.float32, device=self.device)
            with torch.no_grad():
                r = self.torch_model(self.feature_selector(y0)).cpu().numpy()[0]
            cl = self.torch_model.y_clamp.cpu().numpy()
            self._dbg_hits += (np.abs(r) >= 0.95 * cl).astype(int)
            self._dbg_maxabs = np.maximum(self._dbg_maxabs, np.abs(r))
            self._dbg_n += 1
            if self._dbg_n % 20 == 0:
                print(f"[resid-dbg] n={self._dbg_n} "
                      f"res(Dn,Da,Dv)=[{r[1]:+.4f},{r[2]:+.4f},{r[3]:+.4f}] "
                      f"max=[{self._dbg_maxabs[1]:.3f},{self._dbg_maxabs[2]:.4f},{self._dbg_maxabs[3]:.3f}] "
                      f"clamphit=[{self._dbg_hits[1]},{self._dbg_hits[2]},{self._dbg_hits[3]}]/{self._dbg_n}",
                      flush=True)

        # ----- yref / constraints (mpc_solver.py와 동일 흐름) -----
        distance2stop = 0.5 * v
        ocp_solver = self.controller.ocp_solver
        
        for i in range(1, self.N):
            s_target = s + self.target_speed * self.dt * (i + 1)
            if s_target > self.global_path_length:
                s_target = self.global_path_length - distance2stop
            yref = np.array([s_target, 0, 0, self.target_speed, 0, 0, 0, 0])
            ocp_solver.set(i, "yref", yref)
            
            ocp_solver.constraints_set(i, "lh", np.array([
                self.constraint.along_min, self.constraint.alat_min,
                self.model.n_min, self.model.v_min,
                self.model.throttle_min, self.model.delta_min,
            ]))
            ocp_solver.constraints_set(i, "uh", np.array([
                self.constraint.along_max, self.constraint.alat_max,
                self.model.n_max, self.model.v_max,
                self.model.throttle_max, self.model.delta_max,
            ]))
        
        s_target_N = s + self.target_speed * self.Tf
        if s_target_N > self.global_path_length:
            s_target_N = self.global_path_length - distance2stop
        yref_N = np.array([s_target_N, 0, 0, self.target_speed, 0, 0])
        ocp_solver.set(self.N, "yref", yref_N)
        
        x0_arr = np.array(x0)
        ocp_solver.constraints_set(0, "lbx", x0_arr)
        ocp_solver.constraints_set(0, "ubx", x0_arr)
        
        # ----- l4acados solve (residual 자동 적용) -----
        status = self.controller.solve()
        
        # ----- 결과 추출 -----
        x1 = ocp_solver.get(1, "x")
        target_D = float(x1[4])
        target_delta = float(x1[5])
        
        return target_D, target_delta, status


# ============================================================
# Standalone test (CARLA 없이 동작 확인)
# ============================================================
def main():
    """Test: build solver and run a few solves with dummy curvature."""
    from scipy.interpolate import make_interp_spline
    
    Tf, N = 2.0, 20
    target_speed = 6.0
    
    # 직선 path (kappa = 0)
    dense_s = np.linspace(0, 200, 1000)
    kappa_data = np.zeros_like(dense_s)
    
    # acados용 B-spline (degree 3)
    kappa_spline = make_interp_spline(dense_s, kappa_data, k=3)
    coeffs = kappa_spline.c
    knots = kappa_spline.t
    
    # build solver
    solver = MPCSolverResidual(
        coeffs=coeffs, knots=knots,
        model_path="../models/residual_model.pt",
        Tf=Tf, N=N,
        target_speed=target_speed,
        global_path_length=dense_s[-1],
    )
    
    # 매 episode 호출
    solver.update_path(dense_s, kappa_data)
    
    # 몇 step 풀어보기
    print("\n=== Solve test ===")
    x_test = np.array([0.0, 0.0, 0.0, 4.0, 0.0, 0.0])
    
    for k in range(5):
        target_D, target_delta, status = solver.solve(x_test)
        print(f"step {k}: status={status}, D={target_D:+.4f}, delta={np.rad2deg(target_delta):+.2f}deg")
        
        # state 단순 적분 (가상 dynamics)
        x_test[0] += x_test[3] * 0.1   # s += v * dt
        x_test[3] += target_D * 1.0    # v += D * dt (대충)
        x_test[4] = target_D
        x_test[5] = target_delta


if __name__ == "__main__":
    main()