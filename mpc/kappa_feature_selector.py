"""
Custom feature selector for l4acados.
- Maps acados (state, input) to neural network input
- Adds path-dependent kappa(s) on the fly
"""
import torch
import torch.nn as nn
import numpy as np
from scipy.interpolate import make_interp_spline


class KappaAwareFeatureSelector(nn.Module):
    """
    Maps acados full vector y = [x; u] to neural network input.
    
    Input from l4acados:  y = (B, 8) = [s, n, alpha, v, D, delta, derD, derDelta]
    Neural net wants:     X = (B, 6) = [n, alpha, v, D, delta, kappa]
    
    kappa is computed from s using a path-specific spline that must be set 
    before each episode via update_kappa_spline().
    """
    def __init__(self):
        super().__init__()
        # spline params (set per episode)
        self._spline_t = None      # knots
        self._spline_c = None      # coefficients  
        self._spline_k = 3         # degree
        self._s_min = None
        self._s_max = None
    
    def update_kappa_spline(self, dense_s, kappa_values, degree=3):
        """
        Set kappa(s) for current episode.
        
        Args:
            dense_s:      (M,) ndarray, sample s values
            kappa_values: (M,) ndarray, sample kappa values
        """
        spline = make_interp_spline(dense_s, kappa_values, k=degree)
        # Convert spline params to tensors (so PyTorch can backprop through)
        # 근데 우리는 backward 통해 spline을 학습하는 게 아니라 단순 evaluation만 하니까
        # detach해도 되지만, jacobian 계산은 PyTorch 차원에서 됨.
        self._spline_t = torch.from_numpy(spline.t).float()
        self._spline_c = torch.from_numpy(spline.c).float()
        self._s_min = float(dense_s[0])
        self._s_max = float(dense_s[-1])
    
    def _evaluate_kappa(self, s_tensor):
        """
        Evaluate kappa(s) using B-spline.
        Currently uses scipy via numpy roundtrip — acceptable since this is 
        only called per MPC iteration (not per gradient step).
        
        Note: gradient w.r.t. s of kappa is treated as 0 here. 
        For exact gradient propagation, use a differentiable spline.
        """
        # clamp s to valid range
        s_np = s_tensor.detach().cpu().numpy()
        s_np = np.clip(s_np, self._s_min, self._s_max)
        
        # scipy BSpline evaluation
        from scipy.interpolate import BSpline
        bspl = BSpline(
            self._spline_t.cpu().numpy(),
            self._spline_c.cpu().numpy(),
            self._spline_k,
        )
        kappa_np = bspl(s_np)
        
        return torch.from_numpy(np.asarray(kappa_np, dtype=np.float32)).to(s_tensor.device)

    def forward(self, y):
        """
        Args:
            y: (B, 8) = [s, n, alpha, v, D, delta, derD, derDelta]
        
        Returns:
            X: (B, 6) = [n, alpha, v, D, delta, kappa]
        """
        if self._spline_t is None:
            raise RuntimeError("kappa spline not set. Call update_kappa_spline() first.")
        
        s = y[:, 0]
        kappas = self._evaluate_kappa(s)   # (B,)
        
        # [n, alpha, v, D, delta] + kappa
        out = torch.cat([y[:, 1:6], kappas.unsqueeze(-1)], dim=-1)
        return out


# ============================================================
# Standalone test
# ============================================================
def main():
    print("=== Test KappaAwareFeatureSelector ===\n")
    
    # 1. Create dummy path data
    dense_s = np.linspace(0, 100, 200)
    kappa_data = 0.05 * np.sin(2 * np.pi * dense_s / 50.0)   # sinusoidal kappa
    
    # 2. Create selector
    selector = KappaAwareFeatureSelector()
    selector.update_kappa_spline(dense_s, kappa_data)
    
    # 3. Test forward pass
    # y = (B, 8): batch of 5 stages
    y_test = torch.tensor([
        [10.0, 0.05,  0.02, 5.0, 0.3,  0.05, 0.1, 0.02],    # s=10
        [25.0, 0.0,  -0.01, 6.0, 0.25, 0.0,  0.0, 0.0 ],    # s=25
        [50.0, 0.1,   0.03, 7.0, 0.4, -0.05, 0.0, 0.01],    # s=50
        [75.0,-0.05, -0.02, 8.0, 0.35,-0.02, 0.1, 0.0 ],    # s=75
        [99.0, 0.0,   0.0,  4.0, 0.2,  0.0,  0.0, 0.0 ],    # s=99 (near end)
    ], dtype=torch.float32)
    
    out = selector(y_test)
    print(f"Input shape:  {y_test.shape}")
    print(f"Output shape: {out.shape}\n")
    
    print("Input y (s shown):")
    for i, row in enumerate(y_test):
        print(f"  stage {i}: s={row[0]:.1f}")
    
    print("\nOutput X (kappa shown):")
    for i, row in enumerate(out):
        print(f"  stage {i}: [n={row[0]:+.3f}, alpha={row[1]:+.3f}, v={row[2]:.2f}, "
              f"D={row[3]:+.3f}, delta={row[4]:+.3f}, kappa={row[5]:+.5f}]")
    
    # Verify kappa values match the sin pattern
    print("\nKappa verification (selector output vs scipy direct):")
    s_vals = y_test[:, 0].numpy()
    from scipy.interpolate import make_interp_spline
    direct_kappa = make_interp_spline(dense_s, kappa_data, k=3)(s_vals)
    selector_kappa = out[:, 5].numpy()
    err = np.abs(direct_kappa - selector_kappa).max()
    print(f"  Max difference: {err:.2e}")
    if err < 1e-5:
        print("  ✅ Match")
    else:
        print("  ⚠️  Mismatch")


if __name__ == "__main__":
    main()