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
from cost_weights import get_weights

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


class MaskedJacResidualModel(PyTorchResidualModel):
    """잔차 Jacobian에서 지정한 입력 열만 0으로 마스킹 (기본: δ열=idx5).
    폐루프 공선성(corr(δ,n)≈-0.83, R²≈0.97)으로 사실상 unidentified된 ∂g/∂δ만
    제거하고, 잘 학습된 ∂g/∂n·∂g/∂α는 보존한다 -> full FF의 수술적 대안.
    (RESIDUAL_MASK_JAC='5' 또는 '5,6' 처럼 열 인덱스 지정. y=[s,n,α,v,D,δ,derD,derδ])
    잔차 '값' g는 그대로 적용되므로 보정은 유지되고, δ의 confound된 기울기만 뺀다.
    """
    def __init__(self, *args, mask_cols=(5,), **kwargs):
        super().__init__(*args, **kwargs)
        self._mask_cols = list(mask_cols)

    def jacobian(self, y):
        J = super().jacobian(y)          # (rdim, N, nx+nu) numpy, ∂g/∂[x;u]
        J[:, :, self._mask_cols] = 0.0   # confound된 δ열만 제거
        self.current_prediction_dy = J
        return J


class JacScaledResidualModel(PyTorchResidualModel):
    """잔차 Jacobian 전체에 λ(RESIDUAL_JAC_SCALE)를 곱한다.
    λ=1.0 -> full-jac,  λ=0.0 -> FF와 동일.  그 사이로 '얼마나 작은 Jacobian까지
    폐루프가 발산하나'의 안정 임계를 매핑한다. 값 g는 그대로 -> 값 완전 고정,
    Jacobian 크기만 변화(재학습 confound 없음). FF/full의 연속 일반화.
    """
    def __init__(self, *args, jac_scale=1.0, **kwargs):
        super().__init__(*args, **kwargs)
        self._js = float(jac_scale)

    def jacobian(self, y):
        J = super().jacobian(y) * self._js
        self.current_prediction_dy = J
        return J


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
        
        # nominal(acados_settings.py)과 동일한 가중치 모듈 — 공정 비교를 위해 공유.
        Q, R, Qe = get_weights(nu)
        
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
        # l4acados는 nlp_solver_max_iter 만큼 preparation/feedback을 무조건 반복한다
        # (rti_log_residuals가 꺼져 있어 조기 종료가 없다). acados 기본값 100을 그대로
        # 두면 제어 1스텝마다 SQP를 100번 돌아 solve가 ~110ms(주행 중 계측)까지 늘어나
        # 10Hz 예산을 넘긴다. 측정해 보면 10회에서 이미 수렴해 100회 해와 조향 명령이
        # 0.002deg 이내로 같다 -- 나머지 90회는 수렴한 문제를 다시 푸는 순수 낭비다.
        # 10회로 두면 해는 그대로이면서 solve가 88.5ms -> 9.2ms (offline 계측).
        # SQP_ITER=1 은 nominal(SQP_RTI 1회)과 계산량을 맞춘 진짜 RTI 구성(0.99ms)이나
        # 해가 달라지므로 주행 재검증이 필요하다.
        ocp.solver_options.nlp_solver_max_iter = int(os.environ.get("SQP_ITER", "10"))
        
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

        # 잔차 Jacobian 처리 모드:
        #   RESIDUAL_FF=1          -> full feedforward (Jacobian 전체 0)
        #   RESIDUAL_MASK_JAC="5"  -> partial FF (δ열만 0; confound된 ∂g/∂δ 제거)
        #   (둘 다 없으면)         -> full Jacobian
        _FF = os.environ.get("RESIDUAL_FF", "0") == "1"
        _mask = os.environ.get("RESIDUAL_MASK_JAC", "")
        _jscale = os.environ.get("RESIDUAL_JAC_SCALE", "")
        if _FF:
            self.residual_model = FeedforwardResidualModel(
                model=self.torch_model, feature_selector=self.feature_selector,
                use_jacfwd=True)
            print("[MPCSolverResidual] residual mode = FEEDFORWARD (jac=0)", flush=True)
        elif _jscale:
            self.residual_model = JacScaledResidualModel(
                model=self.torch_model, feature_selector=self.feature_selector,
                use_jacfwd=True, jac_scale=float(_jscale))
            print(f"[MPCSolverResidual] residual mode = JAC-SCALED lambda={_jscale} "
                  f"(value fixed, Jacobian x{_jscale}; 1=full 0=FF)", flush=True)
        elif _mask:
            cols = tuple(int(c) for c in _mask.split(","))
            self.residual_model = MaskedJacResidualModel(
                model=self.torch_model, feature_selector=self.feature_selector,
                use_jacfwd=True, mask_cols=cols)
            print(f"[MPCSolverResidual] residual mode = MASKED-JAC cols={cols} "
                  f"(partial FF; δ열이면 5)", flush=True)
        else:
            self.residual_model = PyTorchResidualModel(
                model=self.torch_model, feature_selector=self.feature_selector,
                use_jacfwd=True)
            print("[MPCSolverResidual] residual mode = full (jac)", flush=True)
        
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

        # 격리 실험용 per-step 로그 (RESIDUAL_DBG_LOG=path). 같은 체크포인트로
        # full(use_jacfwd) vs FF(jac=0)를 토글하며 매 스텝 기록:
        #   [s,n,alpha,v,D,delta,  ||B.dg/dx||(참값, FF여부 무관),  Dn,Da,Dv(잔차값),  status]
        # ||B.dg/dx||은 torch_model에서 직접 autograd로 계산 -> full이 쓰고 FF가 버리는
        # 바로 그 Jacobian 크기를 두 실행 모두 동일 정의로 남긴다.
        self._dbglog_path = os.environ.get("RESIDUAL_DBG_LOG", "")
        self._dbglog = []
        if self._dbglog_path:
            import atexit
            atexit.register(self._save_dbglog)

    def _true_bjac_norm(self, x6):
        y = torch.tensor(np.concatenate([np.asarray(x6, float), [0.0, 0.0]])[None, :],
                         dtype=torch.float32, device=self.device, requires_grad=True)
        out = self.torch_model(self.feature_selector(y))   # (1,4) 물리 잔차
        J = torch.zeros(4, 6)
        for k in range(4):
            g, = torch.autograd.grad(out[0, k], y, retain_graph=(k < 3))
            J[k] = g[0, :6]
        return float(np.linalg.norm(B_MATRIX @ J.detach().cpu().numpy()))

    def _save_dbglog(self):
        if self._dbglog_path and self._dbglog:
            np.save(self._dbglog_path, np.array(self._dbglog))
            print(f"[resid-dbg] saved {len(self._dbglog)} rows -> {self._dbglog_path}",
                  flush=True)

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

        if self._dbglog_path:
            with torch.no_grad():
                r = self.torch_model(self.feature_selector(torch.tensor(
                    [[s, n, alpha, v, D, delta, 0.0, 0.0]],
                    dtype=torch.float32, device=self.device))).cpu().numpy()[0]
            jn = self._true_bjac_norm(x0)
            # cols: s,n,alpha,v,D,delta, ||B.dg/dx||, Dn,Da,Dv, target_delta(명령조향), status
            #  -> 발산 개시 때 target_delta가 먼저 과조향으로 튀나(spurious δ-Jac/clamp)
            #     아니면 Dv가 먼저 한 방향으로 쌓이나(payload)를 로그로 격리.
            self._dbglog.append([s, n, alpha, v, D, delta, jn,
                                 float(r[1]), float(r[2]), float(r[3]),
                                 target_delta, int(status)])
        
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