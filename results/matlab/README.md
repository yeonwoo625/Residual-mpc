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

### 분석 결과 `.mat` (2026-08-25 추가, `export_matlab.py`와 별개)

위 표는 원본 배열을 그대로 내보낸 것이고, 아래는 **계산된 분석 결과**다. 각 파일은
전용 파이썬 스크립트가 만들고 전용 `plot_*.m`이 그린다.

| 파일 | 만든 스크립트 | 내용 | 그리기 |
|---|---|---|---|
| `fig_seed_jacobian.mat` | `results/seed_jacobian.py` | 시드 5개 간 값/미분 불일치 [%]. 값 Δn 3.5% vs 미분 78% → **미분은 데이터로 식별되지 않는다** | `plot_seed_jacobian.m` |
| `fig_channel_learnability.mat` | `results/channel_learnability.py` | 채널별 달성치 vs noise ceiling. 횡 1.6% / 종 50.1% 설명 불가 → **종방향은 원리적 학습 불가** | `plot_channel_learnability.m` |
| `figT7_authority.mat` | (세션 스크립트) | 조향 권한 ‖∂n_N/∂δ‖를 3개 주행(ff/js05/js10) 궤적 전체에서 계산. 중첩 struct (`S.js05.auth_nominal` 식) | `plot_figT7_authority.m` |
| `figT7_simple.mat` | 〃 | 위에서 js05 하나만 평탄한 벡터 5개로 뽑은 것 (`s, auth_off, auth_on, jitter_off, jitter_on`) | `plot_figT7_simple.m` |
| `fig_b1_summary.mat` | `results/b1_summary.py` | B1 실험 종합 — 속도 스윕/가중치 스윕/적재 검증/게인×잔차세기. struct 4개 (`speed`, `qn_sweep`, `payload`, `interaction`) | `plot_b1_summary.m` |
| `fig_payload_observability.mat` | `results/payload_observability.py` | 질량이 왜 입력으로 불필요한가 — 상태에서 질량 복원(R²=0.79) + 입력 고정 시 추가 설명력(0.3%) | `plot_payload_observability.m` |
| `fig_safety.mat` | `results/safety_failure_modes.py` | 안전성 — 포화 한계(clamp) 작동 빈도(정상 5~7% vs 불안정 18.6%)와 조향 채터의 상관(r=+0.995), 실패 모드 4종 | `plot_safety.m` |
| `fig_friction.mat` | `results/friction_summary.py` | 저마찰(μ=0.3) — 마찰 사용률 73%에서 잔차 34% 개선, 90%에서는 조향 포화로 전부 이탈. 조향 실효이득 0.65→0.37 | `plot_friction.m` |
| `fig_ddelta.mat` | `results/ddelta_summary.py` | 조향 각속도 제약 민감도 — 57.3→15 deg/s 로 3.8배 강화해도 개선율 유지(58→54%, 17→19%), 채터 39~48% 감소 | `plot_ddelta.m` |
| `fig_highway.mat` + `fig_highway_traj.mat` | `results/highway_summary.py` | 미학습 트랙(USA Highway No.1) — 재학습 없이 적용. nominal 은 차선 여유 초과로 625 m 이탈, residual 은 완주(최대오차 −75%) | `plot_highway.m` |

> ⚠️ `figT7_authority.mat`는 기존 `results/figures/figT7_authority_collapse.png`의 **26% 붕괴를 재현하지 않는다.**
> 그 값은 단일 작동점(12 m/s) 결과이고, 궤적 전체로는 4~141%로 요동친다(중앙값 67~74%).
> 강건한 지표는 붕괴가 아니라 **스텝 간 요동(1.4% → 16%, 11배)**이다.
> 사용 모델: `results/seed_models/seedjac_s0.pt` 계열이 아니라 `mpc/residual_model_nomass_s0.pt`.
> λ-스윕 주행이 어떤 체크포인트로 돌았는지는 로그에 없어 이 모델로 계산했다.

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
| `plot_seed_jacobian.m` | 시드 간 값/미분 불일치 막대 (값 1개 + 미분 6개) |
| `plot_channel_learnability.m` | 채널별 달성치 막대 + noise ceiling 가로선 |
| `plot_figT7_simple.m` | 조향 권한과 스텝 간 요동 (평탄 벡터 버전, 권장) |
| `plot_figT7_authority.m` | 위와 같으나 주행 3개 선택 가능 + 급코너 음영 |
| `plot_b1_summary.m` | B1 종합 — 속도별 nominal vs residual, 가중치 스윕, 적재 검증 (그림 2장) |
| `plot_payload_observability.m` | 질량 관측가능성 3칸 — 예측 산점 / 특징 기여 / 조건부 잉여성 |
| `plot_safety.m` | 포화 빈도 막대 + 채터와의 상관 산점 (이상 징후 지표) |
| `plot_friction.m` | 저마찰 — 완주 조건 성능 비교 + 사용률 대 조향 실효이득 산점 |
| `plot_highway.m` | 미학습 트랙 — 최대오차 대 차선여유 막대 + s별 |n| 궤적 |
| `plot_ddelta.m` | 조향 각속도 제약 민감도 — 개선율 유지 + 채터 감소 막대 |
| `plot_solvetime.m` | MPC 연산시간 — 주행 중 계측 + SQP 반복 스윕 + 해 수렴 (3칸) |
| `plot_extrapolation.m` | 외삽 일반화 — 속도 스윕 + 학습 범위 밖 속도 분포 + 오차·채터 감소 (3칸) |

> **스크립트 규약:** 그림 파일을 저장하지 않는다(`print`/`saveas` 없음). 실행하면 화면에만 띄우고
> 저장은 사용자가 한다. 축 라벨·범례·제목은 **영문**으로 쓴다 — MATLAB 기본 폰트에서 한글이
> 깨질 수 있다. 설명은 파일 상단 주석에 한글로 둔다.

> 색 규약(신규 4개): **파랑 `[0.12 0.47 0.71]` / 주황 `[0.90 0.49 0.13]`**. 기존 그림의 초록/빨강은
> 적록색약 D형에서 ΔE 6.5로 구분되지 않아(파랑/주황은 33.8) 신규 그림부터 바꿨다.

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
