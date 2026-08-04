# MATLAB 형식 데이터 (`results/matlab/`)

모든 실험/그림 데이터를 `.mat`(MATLAB v5, Octave 호환)로 내보낸 것.
재생성: `python3 results/export_matlab.py`

## 파일 구조

| 폴더 | 내용 | 개수 |
|---|---|---:|
| `traj/` | 폐루프 주행 궤적 (nominal vs feedforward residual, 32/48/56t) | 6 |
| `ablation/` | 무게 조건화 ablation 주행 (cond/nomass × 32/48/56t × seed0-2) | 14 |
| `residual_data/` | 잔차 학습·검증 데이터셋 (X 특징 / y 잔차) | 17 |
| `dbglog/` | per-step 진단 로그 (12열, Jacobian·조향권한 진단) | 12 |
| `cloop/` | first-order vs feedforward 비교·마스킹 실험 주행 | 20 |
| `waypoints/` | 기준 경로 (x, y) | 3 |
| `all_data.mat` | 위 전부를 하나의 중첩 struct로 (`traj.ff_32.n` 식 접근) | 1 |

## 변수 규약

### 궤적 파일 (`traj/`, `ablation/`, `cloop/`) — 6열
각 열이 개별 `N×1` double 변수로 저장됨. 원본 행렬은 `raw`, 열 이름은 `cols`, 출처는 `src`.

| 변수 | 뜻 | 단위 |
|---|---|---|
| `s` | 경로거리 | m |
| `n` | 횡추종오차 | m |
| `alpha` | 경로 대비 헤딩각 | rad |
| `v` | 종속도 | m/s |
| `D` | 스로틀 명령 | – |
| `delta` | 조향각 | rad |

### 진단 로그 (`dbglog/`) — 12열
`s, n, alpha, v, D, delta` + `bjac_norm`(‖B·∂g/∂x‖), `res_dn, res_dalpha, res_dv`(MLP 잔차 출력),
`tgt_delta`(MPC가 낸 조향 명령), `status`(acados 솔버 상태, 0=성공).

### 잔차 데이터셋 (`residual_data/`)
- `X` : `N×8` = `[n, alpha, v, D, delta, kappa, mass, cog_x]` (무조건화 데이터는 `N×6`, mass/cog 없음)
- `y` : `N×4` = `[ds, dn, dalpha, dv]` — 잔차 = 실제 다음상태 − 명목모델 예측
- 열 이름은 `X_cols` / `y_cols`, 개별 열도 같은 이름의 `N×1` 변수로 들어있음.
- `mass`는 kg (32000/48000/56000), `cog_x`는 m.

### 경로 (`waypoints/`)
`x`, `y` (`N×1`), 원본 `raw`는 `N×2`.

## 그림 ↔ 데이터 매핑

| 그림 | 데이터 |
|---|---|
| fig1 트랙 위 오차, fig2 s별 \|n\|, fig3 막대, fig5 조향, fig8 다중적재 | `traj/nom_{32,48,56}`, `traj/ff_{32,48,56}` (+ `waypoints/hockenheim_1lap`) |
| fig4 무게→Δn, fig7 무게→스로틀/오차 | `residual_data/hockenheim_mass` |
| fig6 first-order 발산 vs feedforward | `cloop/cloop_fo_*`, `cloop/cloop_ff_*`, `cloop/cloop_first-order` |
| fig9 조건화 ablation | `ablation/*` (cond vs nomass, seed0/1/2) |
| figM1 λ-스윕, figT7 조향권한 붕괴 | `dbglog/ff`, `dbglog/js025`, `dbglog/js05`, `dbglog/js10`, `dbglog/jsneg05` |
| figM2 조향 메커니즘, 인과 Jacobian 식별 | `residual_data/excite_probe12`, `excite_probe8` (능동 조향여기) |
| 조건화 불필요 규명 (CoG·고속) | `residual_data/cog_front48`, `cog_rear48`, `hs14_32`, `hs14_56` |

> figT3~figT6(적재생성·solve time·Jacobian 정확도·시드 일관성)는 학습된 모델(`mpc/residual_model_*.pt`)에서
> 세션 스크립트로 즉석 계산한 값이라 여기에 원본 배열이 없음. 필요하면 요청 — 같은 형식으로 뽑아 추가 가능.

## 요구 버전 / 툴박스

| 항목 | 요구 |
|---|---|
| `.mat` 읽기 | **MATLAB 7 (R14, 2004) 이상** — MAT v5 압축 포맷 |
| 플롯 스크립트 `.m` | **R2013a 이상 권장** (그 이하 미검증). 특수 기능 안 씀 |
| 툴박스 | **불필요** — Statistics/Signal/Curve Fitting 전부 안 씀 |
| 한글 주석·라벨 | **R2020a 이상** 권장 (그 이하는 깨질 수 있음, 아래 참고) |
| Octave | 6 이상이면 동작할 것으로 보이나 미검증 |

의도적으로 피한 것들 (구버전 호환을 위해):
- `quantile` → `pctl.m` 직접 구현 (Statistics Toolbox 회피)
- `movmean`/`smoothdata` → `load_track.m` 안의 이동평균 루프
- `corr` → `corrcoef` (base MATLAB)
- 스크립트 내 로컬 함수(R2016b+) → 전부 별도 `.m` 파일로 분리
- `c.Label.String`(R2014b+) → `ylabel(colorbar, ...)`
- `mean(...,'omitnan')`(R2015a+) → 제거
- `MarkerFaceAlpha`(R2014b+) → 일반 `plot('.')`
- `tiledlayout`/`nexttile`(R2019b+) → `subplot`

**한글 인코딩:** `.m` 파일이 UTF-8인데 R2020a 미만은 시스템 로캘로 읽습니다 (한글 Windows면 CP949).
주석·그림 제목만 깨지고 **계산·플롯 동작에는 지장 없습니다.** 거슬리면 주석/라벨을 영문으로 바꾸면 됩니다.

## 플롯 스크립트 (`.m`)

`results/matlab/`에서 실행 (상대경로로 `.mat`을 읽음). 툴박스 불필요 — base MATLAB만 사용.

| 스크립트 | 재현 대상 |
|---|---|
| `plot_all.m` | 아래 6개 전부 실행 |
| `plot_fig1_track.m` | fig1 — 트랙 위 궤적을 \|n\|으로 색칠 (nominal vs FF) |
| `plot_fig2_error_vs_s.m` | fig2 — s별 \|n\|, 급코너 음영, 3적재 |
| `plot_fig3_bars.m` | fig3/fig8 — 평균/RMS/급코너 RMS 막대 + 표 출력 |
| `plot_fig4_mass_dependence.m` | fig4 — 적재→횡잔차 \|Δn\| + 상관 |
| `plot_fig9_ablation.m` | fig9 — 조건화 O/X 급코너 RMS (시드 점 + paired 차이) |
| `plot_figM1_lambda.m` | figM1/figT7 — λ 스윕 발산, 조향 활동도 |

보조 함수: `load_track.m`(경로→s·psi·kappa), `frenet_to_xy.m`((s,n)→전역좌표),
`corner_mask.m`(급코너 = \|kappa\| 상위 20%), `pctl.m`(분위수).

### 데이터 → 실행할 스크립트 (역방향 색인)

| 가진 데이터 | 실행 | 스크립트가 실제로 읽는 파일 |
|---|---|---|
| `traj/nom_*`, `traj/ff_*` | `plot_fig1_track` (트랙 위 색칠, 파일 상단 `MASS` 로 적재 선택)<br>`plot_fig2_error_vs_s` (s별 \|n\|, 3적재 한 번에)<br>`plot_fig3_bars` (막대 + 표) | `traj/{nom,ff}_{32,48,56}.mat` + `waypoints/hockenheim_1lap.mat` |
| `ablation/cond_*`, `ablation/nomass_*` | `plot_fig9_ablation` | `ablation/{cond,nomass}_{32,56}_s{0,1,2}.mat` (48t는 안 씀) |
| `residual_data/hockenheim_mass` | `plot_fig4_mass_dependence` | 그 파일 하나 |
| `dbglog/ff`, `js025`, `js05`, `js10`, `jsneg05` | `plot_figM1_lambda` | 그 5개 |
| `waypoints/*` | 단독 스크립트 없음 — `load_track` 이 내부에서 사용 | — |
| `cloop/*` | 전용 스크립트 없음 (아래 스니펫) | — |
| `residual_data/` 나머지 (`cog_*`, `hs14_*`, `excite_probe*`, `truck_*` …) | 전용 스크립트 없음 (아래 스니펫) | — |
| `dbglog/` 48t 계열 (`jr05_48`, `mask03_48` …) | 전용 스크립트 없음 — `plot_figM1_lambda` 의 `runs` 목록에 이름만 추가하면 그대로 그려짐 | — |

전용 스크립트가 없는 데이터용 최소 스니펫:

```matlab
% cloop/ (6열 궤적) — 아무 주행이나 같은 방식으로
r = load('cloop/cloop_fo_d1.mat');
plot(r.s, abs(r.n)); xlabel('s [m]'); ylabel('|n| [m]');

% residual_data/ (X/y 데이터셋) — 잔차 채널 분포
d = load('residual_data/hs14_56.mat');
histogram(d.dn, 60); xlabel('\Deltan [m/step]');
plot(d.v, d.dn, '.');            % 속도 의존성
```

> **급코너 정의 주의:** 여기서는 기준 경로 곡률의 상위 20%로 정의했다(`verify_decorrelation.py` 규약).
> 원본 PNG를 만든 세션 스크립트와 정의가 완전히 같지는 않아 **급코너 절대값이 ~20% 다를 수 있다**
> (예: 32t 급코너 RMS 0.215 vs 원본 0.281). 전체 평균/RMS와 개선율, 결론은 동일하다.

## 사용 예 (MATLAB / Octave)

```matlab
% 1) 개별 파일
ff  = load('traj/ff_32.mat');
nom = load('traj/nom_32.mat');
plot(ff.s, abs(ff.n), nom.s, abs(nom.n)); legend('FF residual','nominal');
xlabel('s [m]'); ylabel('|n| [m]');

% 2) 통합 struct
D = load('all_data.mat');
mean(abs(D.traj.ff_32.n))     % 0.0959
mean(abs(D.traj.nom_32.n))    % 0.3031

% 3) 데이터셋
d = load('residual_data/hockenheim_mass.mat');
corr(d.mass, abs(d.dn))       % 무게 ↔ 횡잔차 상관
```
