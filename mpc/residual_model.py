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


class ConditionedResidualModel(nn.Module):
    """
    Payload-conditioned residual MLP (적재 무게·정적 무게중심 조건화).

    입력은 파이프라인 호환을 위해 '단일 벡터'지만, 안에서 상태/조건으로 나눔:
        x = [ state(6) ; context(2) ]
        state   = [n, alpha, v, D, delta, kappa]   (매순간 변함)
        context = [m, x_cog]                        (적재당 고정: 무게, 정적 CoG)
    출력: [ds, dn, dalpha, dv]  (4)

    mode (ablation용):
      'concat' : 상태+조건 붙여서 MLP            (baseline)
      'branch' : 상태 브랜치 + 조건 브랜치 → concat → head
      'film'   : 조건이 상태 특징을 변조 γ⊙h+β    (추천)
    """
    def __init__(self, state_dim=6, context_dim=2, output_dim=4,
                 hidden_dim=64, mode='film'):
        super().__init__()
        self.state_dim = state_dim
        self.context_dim = context_dim
        self.mode = mode
        A = nn.Tanh   # 매끄러운 Jacobian (MPC 안정성)

        if mode == 'concat':
            self.net = nn.Sequential(
                nn.Linear(state_dim + context_dim, hidden_dim), A(),
                nn.Linear(hidden_dim, hidden_dim), A(),
                nn.Linear(hidden_dim, output_dim),
            )
        elif mode == 'branch':
            self.state_net = nn.Sequential(nn.Linear(state_dim, hidden_dim), A())
            self.ctx_net   = nn.Sequential(nn.Linear(context_dim, hidden_dim), A())
            self.head = nn.Sequential(
                nn.Linear(2 * hidden_dim, hidden_dim), A(),
                nn.Linear(hidden_dim, output_dim),
            )
        elif mode == 'film':
            self.state_in  = nn.Linear(state_dim, hidden_dim)
            self.ctx_net   = nn.Sequential(nn.Linear(context_dim, hidden_dim), A())
            self.film      = nn.Linear(hidden_dim, 2 * hidden_dim)  # -> (gamma, beta)
            self.state_mid = nn.Linear(hidden_dim, hidden_dim)
            self.head      = nn.Linear(hidden_dim, output_dim)
            self.act = A()
            # 초기엔 변조 없음(γ=1, β=0) -> 무조건화 모델처럼 안정적으로 시작
            nn.init.zeros_(self.film.weight)
            with torch.no_grad():
                self.film.bias[:hidden_dim] = 1.0
                self.film.bias[hidden_dim:] = 0.0
        else:
            raise ValueError(f"unknown mode: {mode}")

    def forward(self, x):
        xs = x[..., :self.state_dim]              # 상태
        xc = x[..., self.state_dim:]              # 조건 (m, cog)
        if self.mode == 'concat':
            return self.net(x)
        if self.mode == 'branch':
            h = torch.cat([self.state_net(xs), self.ctx_net(xc)], dim=-1)
            return self.head(h)
        # film
        h = self.act(self.state_in(xs))           # 상태 특징
        gamma, beta = self.film(self.ctx_net(xc)).chunk(2, dim=-1)
        h = gamma * h + beta                       # ★ 조건이 상태 특징을 변조
        h = self.act(self.state_mid(h))
        return self.head(h)