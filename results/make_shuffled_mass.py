#!/usr/bin/env python3
"""
플라시보 데이터 생성: 무게/CoG 열을 샘플 간 무작위로 섞는다.

목적 — 조건화 모델이 학습 적재에서 보인 이득(-34%)이
  (a) 무게 '정보' 때문인지,
  (b) 입력 차원 2개·파라미터 128개가 늘어난 효과인지
를 가른다. 섞은 열은 분포(평균·표준편차)는 그대로지만 정보량이 0이다.
  - 가짜 무게가 blind 와 비슷 -> 이득은 무게 정보 덕분 (조건화 정당)
  - 가짜 무게가 조건화와 비슷 -> 이득은 무게와 무관 (조건화 주장 붕괴)

출력: data/hockenheim_shufmass.npz  (X 8열, mass/cog만 섞임)
"""
import os
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(os.path.dirname(ROOT), "mpc_host", "hockenheim_mass.npz")
OUT = os.path.join(ROOT, "data", "hockenheim_shufmass.npz")

d = np.load(SRC)
X, y = d['X'].copy(), d['y'].copy()

rng = np.random.default_rng(12345)
perm = rng.permutation(len(X))
X[:, 6:8] = X[perm, 6:8]          # (mass, cog) 쌍을 통째로 섞음

np.savez(OUT, X=X, y=y)

orig = d['X'][:, 6]
print(f"원본  mass 평균 {orig.mean():.0f} std {orig.std():.0f}  "
      f"corr(mass, |dn|) = {np.corrcoef(orig, np.abs(y[:,1]))[0,1]:+.4f}")
print(f"섞음  mass 평균 {X[:,6].mean():.0f} std {X[:,6].std():.0f}  "
      f"corr(mass, |dn|) = {np.corrcoef(X[:,6], np.abs(y[:,1]))[0,1]:+.4f}")
print(f"동일 위치 유지 비율 {np.mean(perm == np.arange(len(X)))*100:.2f}%")
print(f"저장: {OUT}  X{X.shape} y{y.shape}")
