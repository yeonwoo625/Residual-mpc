"""
Residual model wrapper for l4acados integration.
- Loads trained MLP from checkpoint
- Embeds normalization (input + output)
- Provides clean forward pass: raw input → raw residual output
"""
import os
import torch
import torch.nn as nn
import numpy as np

from residual_model import ResidualModel, ConditionedResidualModel


class NormalizedResidualModel(nn.Module):
    """
    Wraps a trained MLP with input/output normalization.
    
    Input:  raw 6-dim [n, alpha, v, D, delta, kappa]
    Output: raw 4-dim [Δs, Δn, Δalpha, Δv]
    """
    def __init__(self, base_mlp, X_mean, X_std, y_mean, y_std, clamp=None, scale=1.0):
        super().__init__()
        self.mlp = base_mlp
        # scale<1 shrinks the residual AND its Jacobian -> gentler, more stable
        # MPC (the MLP's rough Jacobian is what destabilizes the closed loop).
        self.scale = float(scale)
        # register_buffer: 학습 안 되지만 device 이동 같이 따라감
        self.register_buffer('X_mean', torch.tensor(X_mean, dtype=torch.float32))
        self.register_buffer('X_std',  torch.tensor(X_std,  dtype=torch.float32))
        self.register_buffer('y_mean', torch.tensor(y_mean, dtype=torch.float32))
        self.register_buffer('y_std',  torch.tensor(y_std,  dtype=torch.float32))
        # Physical per-step (dt=0.1s) residual bounds [Δs,Δn,Δα,Δv]. Off-
        # distribution inputs make a small MLP extrapolate to garbage; this hard
        # clamp keeps the residual MPC from being destabilized. Identity inside
        # the bounds (normal operation), saturates outside.
        if clamp is None:
            clamp = [0.2, 0.1, 0.05, 0.4]
        self.register_buffer('y_clamp', torch.tensor(clamp, dtype=torch.float32))

    def forward(self, x):
        """
        x: (B, 6) raw input
        return: (B, 4) raw residual (clamped to physical per-step bounds)
        """
        x_norm = (x - self.X_mean) / self.X_std
        y_norm = self.mlp(x_norm)
        y = (y_norm * self.y_std + self.y_mean) * self.scale
        y = torch.clamp(y, -self.y_clamp, self.y_clamp)
        return y


def load_normalized_model(checkpoint_path, device='cpu', scale=None):
    """
    Load checkpoint and return NormalizedResidualModel ready for inference.
    scale: residual multiplier (default reads env RESIDUAL_SCALE, else 1.0).
    """
    if scale is None:
        scale = float(os.environ.get("RESIDUAL_SCALE", "1.0"))
    print(f"[residual] output scale = {scale}", flush=True)
    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=False)
    
    cfg = ckpt['config']
    norm = ckpt['norm']

    # base MLP: 조건화(context_dim>0)면 ConditionedResidualModel, 아니면 plain
    ctx_dim = cfg.get('context_dim', 0) or 0
    if ctx_dim > 0:
        base = ConditionedResidualModel(
            state_dim=cfg.get('state_dim', 6),
            context_dim=ctx_dim,
            output_dim=cfg['output_dim'],
            hidden_dim=cfg['hidden_dim'],
            mode=cfg.get('mode', 'concat'),
        )
        print(f"[residual] CONDITIONED model: mode={cfg.get('mode')}, "
              f"context_dim={ctx_dim} (input={cfg['input_dim']})", flush=True)
    else:
        base = ResidualModel(
            input_dim=cfg['input_dim'],
            output_dim=cfg['output_dim'],
            hidden_dim=cfg['hidden_dim'],
            n_layers=cfg['n_layers'],
        )
    base.load_state_dict(ckpt['state_dict'])
    
    # normalized wrapper
    model = NormalizedResidualModel(
        base,
        X_mean=norm['X_mean'],
        X_std=norm['X_std'],
        y_mean=norm['y_mean'],
        y_std=norm['y_std'],
        scale=scale,
    )
    model.to(device)
    model.eval()
    
    return model


# ============================================================
# Standalone test
# ============================================================
def main():
    """Test: load model, run forward pass, check outputs."""
    ckpt_path = "../models/residual_model.pt"
    model = load_normalized_model(ckpt_path, device='cpu')
    print(f"Model loaded from {ckpt_path}")
    print(f"Total parameters: {sum(p.numel() for p in model.parameters())}")
    
    # Test cases (raw input units)
    test_inputs = np.array([
        # n     alpha   v    D     delta   kappa
        [ 0.0,  0.0,    5.0, 0.3,  0.0,    0.0   ],   # 직선 정상 주행
        [ 0.05, 0.02,   6.0, 0.25, 0.05,   0.005 ],   # 약곡선
        [-0.1, -0.05,   7.0, 0.4, -0.1,    0.05  ],   # 반대 곡선
        [ 0.0,  0.0,    8.0, 0.35, 0.0,    0.149 ],   # 급곡선
    ], dtype=np.float32)
    
    print("\n=== Test forward pass ===")
    x_t = torch.from_numpy(test_inputs)
    with torch.no_grad():
        residual = model(x_t).numpy()
    
    print("Input (raw):")
    print("    n      alpha    v       D       delta   kappa")
    for row in test_inputs:
        print("  " + "  ".join(f"{v:+.4f}" for v in row))
    print("\nOutput residual (raw):")
    print("    Δs       Δn       Δalpha   Δv")
    for row in residual:
        print("  " + "  ".join(f"{v:+.5f}" for v in row))
    
    # Sanity check: model should output something non-trivial
    if np.abs(residual).max() < 1e-5:
        print("\n⚠️  WARNING: residual outputs are all near zero. Something wrong?")
    else:
        print("\n✅ Model produces non-trivial residuals")
    
    # Check the systematic Δv bias (should be around -0.085 from training stats)
    dv_avg = residual[:, 3].mean()
    print(f"\nMean Δv across test cases: {dv_avg:+.4f}")
    print(f"  (Expected near -0.085 if drivetrain bias was learned)")


if __name__ == "__main__":
    main()