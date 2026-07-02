"""
Train residual model with supervised regression.
"""
import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from residual_model import ResidualModel, count_parameters


def load_dataset(npz_path, val_ratio=0.2, seed=42, split_mode='time'):
    """
    데이터 로드 + train/val split.
    
    split_mode:
      'random' : 무작위 분할 (data leakage 가능, 시계열 데이터엔 부적절)
      'time'   : 시간 순서 분할 (마지막 val_ratio를 val로) - 권장
    """
    data = np.load(npz_path)
    X = data['X']
    y = data['y']
    
    N = len(X)
    n_val = int(N * val_ratio)
    
    if split_mode == 'random':
        rng = np.random.default_rng(seed)
        idx = rng.permutation(N)
        X = X[idx]
        y = y[idx]
        X_train, X_val = X[n_val:], X[:n_val]
        y_train, y_val = y[n_val:], y[:n_val]
    elif split_mode == 'time':
        # 시간 순서 보존: 앞 80% train, 마지막 20% val
        X_train, X_val = X[:N - n_val], X[N - n_val:]
        y_train, y_val = y[:N - n_val], y[N - n_val:]
    else:
        raise ValueError(f"Unknown split_mode: {split_mode}")
    
    print(f"Split mode: {split_mode}")
    return (X_train, y_train), (X_val, y_val)


def fit_normalizer(X_train, y_train):
    """평균/표준편차 계산 (train 데이터로만)."""
    return {
        'X_mean': X_train.mean(axis=0),
        'X_std':  X_train.std(axis=0) + 1e-6,
        'y_mean': y_train.mean(axis=0),
        'y_std':  y_train.std(axis=0) + 1e-6,
    }


def normalize(arr, mean, std):
    return (arr - mean) / std


def unnormalize(arr, mean, std):
    return arr * std + mean


def train(args):
    # ----- load data -----
    (X_train, y_train), (X_val, y_val) = load_dataset(
        args.data, val_ratio=0.2, split_mode=args.split_mode)
    print(f"Train: {len(X_train)} samples, Val: {len(X_val)} samples")
    
    # ----- normalize -----
    norm = fit_normalizer(X_train, y_train)
    print(f"\nNormalization stats:")
    print(f"  X_mean: {norm['X_mean']}")
    print(f"  X_std:  {norm['X_std']}")
    print(f"  y_mean: {norm['y_mean']}")
    print(f"  y_std:  {norm['y_std']}")
    
    Xn_train = normalize(X_train, norm['X_mean'], norm['X_std'])
    yn_train = normalize(y_train, norm['y_mean'], norm['y_std'])
    Xn_val   = normalize(X_val,   norm['X_mean'], norm['X_std'])
    yn_val   = normalize(y_val,   norm['y_mean'], norm['y_std'])
    
    # ----- to torch -----
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"\nDevice: {device}")
    
    Xn_train_t = torch.tensor(Xn_train, dtype=torch.float32, device=device)
    yn_train_t = torch.tensor(yn_train, dtype=torch.float32, device=device)
    Xn_val_t   = torch.tensor(Xn_val,   dtype=torch.float32, device=device)
    yn_val_t   = torch.tensor(yn_val,   dtype=torch.float32, device=device)
    
    train_loader = DataLoader(
        TensorDataset(Xn_train_t, yn_train_t),
        batch_size=args.batch_size, shuffle=True,
    )
    
    # ----- model -----
    model = ResidualModel(
        input_dim=6, output_dim=4,
        hidden_dim=args.hidden, n_layers=args.layers,
    ).to(device)
    print(f"\nModel: {count_parameters(model)} trainable parameters")
    
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr,
                                 weight_decay=args.weight_decay)
    criterion = nn.MSELoss()
    print(f"weight_decay={args.weight_decay}  jac_reg={args.jac_reg}")
    
    # ----- training loop -----
    train_losses, val_losses = [], []
    best_val_loss = float('inf')
    
    for epoch in range(args.epochs):
        model.train()
        epoch_train_loss = 0.0
        n_batches = 0
        for xb, yb in train_loader:
            if args.jac_reg > 0:
                xb = xb.requires_grad_(True)
            pred = model(xb)
            loss = criterion(pred, yb)
            if args.jac_reg > 0:
                # penalize ||d(residual)/d(state)||^2 -> smoother Jacobian ->
                # stable residual MPC (the rough Jacobian is what destabilizes).
                jac_pen = 0.0
                for k in range(pred.shape[1]):
                    g = torch.autograd.grad(
                        pred[:, k].sum(), xb,
                        create_graph=True, retain_graph=True)[0]
                    jac_pen = jac_pen + (g ** 2).sum(dim=1).mean()
                loss = loss + args.jac_reg * jac_pen
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            epoch_train_loss += loss.item()
            n_batches += 1
        epoch_train_loss /= n_batches
        
        model.eval()
        with torch.no_grad():
            val_pred = model(Xn_val_t)
            val_loss = criterion(val_pred, yn_val_t).item()
        
        train_losses.append(epoch_train_loss)
        val_losses.append(val_loss)
        
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
        
        if (epoch + 1) % 10 == 0:
            print(f"Epoch {epoch+1:3d}/{args.epochs}: "
                  f"train {epoch_train_loss:.5f}, val {val_loss:.5f}")
    
    # ----- save best model -----
    model.load_state_dict(best_state)
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'models')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'residual_model.pt')
    torch.save({
        'state_dict': best_state,
        'norm': norm,
        'config': {
            'input_dim': 6, 'output_dim': 4,
            'hidden_dim': args.hidden, 'n_layers': args.layers,
        }
    }, out_path)
    print(f"\nSaved best model to {out_path}")
    print(f"Best val loss: {best_val_loss:.5f}")
    
    # ----- per-channel evaluation in original units -----
    model.eval()
    with torch.no_grad():
        val_pred_n = model(Xn_val_t).cpu().numpy()
    val_pred = unnormalize(val_pred_n, norm['y_mean'], norm['y_std'])
    
    print("\n=== Per-channel RMSE on validation (original units) ===")
    channel_names = ['Δs', 'Δn', 'Δalpha', 'Δv']
    for i, name in enumerate(channel_names):
        residual_rmse_target = np.sqrt(np.mean(y_val[:, i]**2))   # 학습 안 한 baseline
        residual_rmse_pred   = np.sqrt(np.mean((y_val[:, i] - val_pred[:, i])**2))
        print(f"  {name:10s}: target RMS {residual_rmse_target:.5f}, "
              f"pred-target RMSE {residual_rmse_pred:.5f}, "
              f"reduction {(1 - residual_rmse_pred/residual_rmse_target)*100:5.1f}%")
    
    # ----- plots -----
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    axes[0].plot(train_losses, label='train')
    axes[0].plot(val_losses, label='val')
    axes[0].set_xlabel("epoch")
    axes[0].set_ylabel("MSE loss (normalized)")
    axes[0].set_yscale('log')
    axes[0].legend(); axes[0].grid()
    axes[0].set_title("Training curves")
    
    # scatter: predicted vs actual for Δn (가장 흥미로운 채널)
    axes[1].scatter(y_val[:, 1], val_pred[:, 1], alpha=0.3, s=8)
    lim = max(abs(y_val[:, 1]).max(), abs(val_pred[:, 1]).max())
    axes[1].plot([-lim, lim], [-lim, lim], 'k--', alpha=0.5)
    axes[1].set_xlabel("actual Δn")
    axes[1].set_ylabel("predicted Δn")
    axes[1].grid()
    axes[1].set_title("Δn: predicted vs actual (val)")
    
    plt.tight_layout()
    out_plot = os.path.join(out_dir, 'training_curve.png')
    plt.savefig(out_plot, dpi=120)
    print(f"Saved training plot to {out_plot}")
    plt.show()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--data", type=str, default="../data/dataset_small.npz")
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--weight-decay", type=float, default=0.0)
    p.add_argument("--jac-reg", type=float, default=0.0,
                   help="penalty on ||d(residual)/d(state)||^2 (smoother model)")
    p.add_argument("--hidden", type=int, default=64)
    p.add_argument("--layers", type=int, default=3)
    p.add_argument("--split-mode", type=str, default='time',
                   choices=['random', 'time'],
                   help="train/val split mode")
    args = p.parse_args()
    train(args)