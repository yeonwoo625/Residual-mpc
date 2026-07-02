"""
Residual dynamics model.
Input:  observation [n, alpha, v, D, delta, kappa]    (6 dim)
Output: residual    [Δs, Δn, Δalpha, Δv]              (4 dim)

Trained with supervised regression (MSE loss).
"""
import torch
import torch.nn as nn


class ResidualModel(nn.Module):
    """
    Simple MLP for residual learning.
    Input/output normalization is handled outside (in train script).
    """
    def __init__(self, input_dim=6, output_dim=4, hidden_dim=64, n_layers=3):
        super().__init__()
        layers = []
        in_dim = input_dim
        for i in range(n_layers - 1):
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(nn.Tanh())
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, output_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def count_parameters(model):
    return sum(p.numel() for p in model.parameters() if p.requires_grad)