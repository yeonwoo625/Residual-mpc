# 시드별 모델 (`seedjac_s0~4.pt`)

**미분 비식별성 분석 전용.** 주행 실험에는 쓰지 않는다.

같은 데이터·같은 구조·같은 하이퍼파라미터로 **난수 초기값(seed)만** 바꿔 5개를 학습했다.
다른 것이 오직 출발점 하나뿐이므로, 결과가 갈리면 그것은 순전히 우연이다.

```
데이터   results/ablation/hockenheim_nomass.npz  (8,088 샘플, 입력 6차원)
구조     64-64, Tanh, 출력 4  (θ 4,868개)
학습     200 epochs, batch 64, lr 1e-3, weight_decay 0, jac_reg 0, 시간순 분할
seed     0, 1, 2, 3, 4        ← 이것만 다름
```

재현:

```bash
python3 mpc/train_residual.py --data results/ablation/hockenheim_nomass.npz \
    --epochs 200 --batch-size 64 --lr 1e-3 --hidden 64 --layers 3 \
    --split-mode time --seed <S>
# train_residual.py는 항상 models/residual_model.pt에 저장하므로 실행 후 옮겨야 한다
```

분석: `python3 results/seed_jacobian.py` → `results/matlab/fig_seed_jacobian.mat`

## 결과

| | 시드 간 불일치 |
|---|---|
| 값 Δn | **3.5%** |
| 미분 ∂Δn/∂n | 85.2% |
| 미분 ∂Δn/∂α | 77.4% |
| 미분 ∂Δn/∂v | 89.1% |
| 미분 ∂Δn/∂D | **99.6%** |
| 미분 ∂Δn/∂δ | 50.5% |
| 미분 ∂Δn/∂κ | 67.6% |

**데이터는 값을 식별하지만 미분은 식별하지 못한다.** 학습된 미분은 트럭의 물리가 아니라
초기 난수가 결정한 값이며, 이것이 first-order 주입(λ≠0)이 λ의 크기·부호와 무관하게
발산하는 근본 원인이다. 이 증거는 MPC를 전혀 사용하지 않으므로 "솔버/튜닝 문제"라는
반박이 성립하지 않는다.

## 기존 모델과의 관계

`mpc/residual_model_nomass_s0~2.pt`(주행 실험용)와는 **별개**다. 그쪽은 학습 조건이
동일하다는 보장이 없어 "초기값만 다르다"는 주장에 쓸 수 없으므로, 이 분석을 위해
5개를 같은 레시피로 새로 학습했다. 기존 체크포인트는 건드리지 않았다.
