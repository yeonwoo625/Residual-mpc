#!/usr/bin/env python3
"""
기준 경로를 경로에 수직인 방향으로 평행 이동한다.

Highway No.1 은 갓길 폭이 좌우 비대칭이고(왼쪽 0.75~4.50 m, 오른쪽 0.21~1.99 m)
구간마다 변한다. IPGDriver 로 녹화한 기준 경로는 우측에 치우쳐 있어, nominal 이
s=625 m 에서 우측 타이어가 도로를 벗어나며 SIM_ABORT 가 났다(횡편차 0.46 m 에서).

경로를 왼쪽으로 밀면 우측 여유가 그만큼 늘어난다. 재녹화 없이 좌표만 옮긴다.

    X' = X - sin(psi) * d
    Y' = Y + cos(psi) * d        d > 0 = 진행방향 기준 왼쪽

주의: 도로 형상을 모르고 미는 것이므로 너무 크게 밀면 반대쪽(왼쪽)으로 나간다.
왼쪽 갓길이 좁아지는 구간이 있는지 확인하려면 결국 주행해 봐야 한다.
1.0 m 정도부터 시도하고, 이탈하면 줄인다.

사용:
    python3 scripts/shift_path.py <입력.npy> <출력.npy> <이동량 m>
예:
    python3 scripts/shift_path.py \\
        /home/vilab/CarMaker/mpc_host/highway_waypoints.npy \\
        /home/vilab/CarMaker/mpc_host/highway_waypoints_L1.npy  1.0
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "mpc"))
from reference_utils import compute_path_spline                  # noqa: E402


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    src, dst, d = sys.argv[1], sys.argv[2], float(sys.argv[3])

    w = np.load(src)
    pi = compute_path_spline(np.asarray(w, dtype=float))
    s_ref = pi["dense_s"]
    # phi 는 +-pi 로 감겨 있다. unwrap 해야 보간이 튀지 않는다.
    psi = np.unwrap(pi["phi"])

    # 각 웨이포인트의 s 를 찾아 그 지점의 접선 방향으로 수직 이동
    x_ref = pi["x_spline"](s_ref)
    y_ref = pi["y_spline"](s_ref)
    sw = np.zeros(len(w))
    for i, p in enumerate(w):
        sw[i] = s_ref[np.argmin((x_ref - p[0]) ** 2 + (y_ref - p[1]) ** 2)]
    ps = np.interp(sw, s_ref, psi)

    out = np.stack([w[:, 0] - np.sin(ps) * d,
                    w[:, 1] + np.cos(ps) * d], 1)
    np.save(dst, out)

    pi2 = compute_path_spline(out)
    print(f"입력  {src}\n      {len(w)} 점, 길이 {pi['dense_s'][-1]:.1f} m, "
          f"최소반경 {1/np.abs(pi['kappa']).max():.1f} m")
    print(f"이동  {d:+.2f} m ({'왼쪽' if d > 0 else '오른쪽'})")
    print(f"출력  {dst}\n      {len(out)} 점, 길이 {pi2['dense_s'][-1]:.1f} m, "
          f"최소반경 {1/np.abs(pi2['kappa']).max():.1f} m")
    print(f"\n실제 이동 거리 (첫 점): {np.hypot(*(out[0]-w[0])):.3f} m")


if __name__ == "__main__":
    main()
