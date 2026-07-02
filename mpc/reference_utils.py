"""
Reference path utilities.
- generate_carla_waypoints: CARLA map에서 차선 따라 waypoint 자동 생성
- compute_path_spline: waypoint -> arc length s, curvature kappa, x/y splines
- compute_kappa_spline_for_acados: kappa(s) B-spline coefs/knots (acados용)
- cartesian_to_frenet / frenet_to_cartesian: 좌표 변환
"""
import numpy as np
import scipy.interpolate as interp


def generate_carla_waypoints(world, start_location, distance=2.0, max_length=200.0):
    """
    CARLA 맵에서 시작 위치부터 차선을 따라 waypoint를 자동 생성.

    Args:
        world          : CARLA world object
        start_location : carla.Location (시작점)
        distance       : waypoint 간 거리 [m]
        max_length     : 전체 경로 최대 길이 [m]

    Returns:
        waypoints : ndarray (N, 2) [x, y]
    """
    carla_map = world.get_map()
    wp = carla_map.get_waypoint(start_location, project_to_road=True)
    
    pts = [(wp.transform.location.x, wp.transform.location.y)]
    total_length = 0.0
    
    while total_length < max_length:
        next_wps = wp.next(distance)
        if not next_wps:
            break
        wp = next_wps[0]   # 첫 번째 옵션 (분기에서 직진 우선)
        pts.append((wp.transform.location.x, wp.transform.location.y))
        total_length += distance
    
    return np.array(pts)


def compute_path_spline(waypoints):
    """
    Waypoint (N, 2) -> arc length s, x/y splines, dense kappa.
    참고 레포 parseReference 포팅 (ROS 의존성 제거).

    Returns dict:
        x_spline   : CubicSpline x(s)
        y_spline   : CubicSpline y(s)
        s_total    : 전체 길이
        dense_s    : 균등 sampled s (간격 ≈ 0.1m)
        phi        : dense_s에서의 heading angle
        kappa      : dense_s에서의 curvature
        ds_at_wp   : 각 waypoint의 누적 arc length
    """
    xs = waypoints[:, 0].tolist()
    ys = waypoints[:, 1].tolist()

    # 시작/끝 슬로프 (clamped boundary)
    x1, x2 = xs[0], xs[1]
    y1, y2 = ys[0], ys[1]
    dist_s = np.hypot(x2 - x1, y2 - y1)
    xe, xee = xs[-1], xs[-2]
    ye, yee = ys[-1], ys[-2]
    dist_e = np.hypot(xe - xee, ye - yee)
    x_slope_start = (x2 - x1) / dist_s
    y_slope_start = (y2 - y1) / dist_s
    x_slope_end   = (xe - xee) / dist_e
    y_slope_end   = (ye - yee) / dist_e

    # 누적 arc length
    ds = [0.0]
    for i in range(1, len(xs)):
        ds.append(ds[-1] + np.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1]))

    x_spline = interp.CubicSpline(
        ds, xs, bc_type=((1, x_slope_start), (1, x_slope_end))
    )
    y_spline = interp.CubicSpline(
        ds, ys, bc_type=((1, y_slope_start), (1, y_slope_end))
    )

    density = 0.1  # dense sampling 간격
    n_dense = max(int(ds[-1] / density), 100)
    dense_s = np.linspace(ds[0], ds[-1], n_dense)

    dx_ds  = x_spline.derivative()(dense_s)
    dy_ds  = y_spline.derivative()(dense_s)
    dx2_ds = x_spline.derivative(nu=2)(dense_s)
    dy2_ds = y_spline.derivative(nu=2)(dense_s)

    phi = np.arctan2(dy_ds, dx_ds)
    kappa = (dx_ds * dy2_ds - dy_ds * dx2_ds) / (dx_ds**2 + dy_ds**2)**1.5

    return {
        "x_spline": x_spline,
        "y_spline": y_spline,
        "s_total": ds[-1],
        "dense_s": dense_s,
        "phi": phi,
        "kappa": kappa,
        "ds_at_wp": np.array(ds),
    }


def compute_kappa_spline_for_acados(dense_s, kappa, degree=3):
    """
    kappa(s)의 B-spline 표현을 acados용 (coeffs, knots)로 추출.
    참고 레포 mpc_controller.py 라인 270-272 포팅.
    """
    kappa_spline = interp.make_interp_spline(dense_s, kappa, k=degree)
    return kappa_spline.c, kappa_spline.t


# ============================================================
#  Cartesian <-> Frenet 변환
# ============================================================

def wrap_to_pi(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def cartesian_to_frenet(X, Y, psi, path_info):
    """
    (X, Y, psi) -> (s, n, alpha)
    가장 가까운 segment에서 foot-of-perpendicular로 정확하게 변환.

    Args:
        X, Y, psi : 차량 Cartesian 위치/heading
        path_info : compute_path_spline()의 반환 dict

    Returns:
        s, n, alpha
    """
    x_sp = path_info["x_spline"]
    y_sp = path_info["y_spline"]
    s_grid = np.linspace(0, path_info["s_total"], 1000)
    xs = x_sp(s_grid)
    ys = y_sp(s_grid)
    
    # 1. 가장 가까운 grid 점
    d2 = (xs - X)**2 + (ys - Y)**2
    idx = int(np.argmin(d2))

    # 2. 인접 두 segment에서 foot-of-perpendicular
    candidates = []
    if idx > 0: candidates.append(idx - 1)
    if idx < len(s_grid) - 1: candidates.append(idx)
    
    best = None
    for i in candidates:
        P0 = np.array([xs[i],   ys[i]])
        P1 = np.array([xs[i+1], ys[i+1]])
        seg = P1 - P0
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-9:
            continue
        t = np.dot(np.array([X, Y]) - P0, seg) / (seg_len**2)
        t = np.clip(t, 0.0, 1.0)
        foot = P0 + t * seg
        d = np.linalg.norm(np.array([X, Y]) - foot)
        s_val = (1 - t) * s_grid[i] + t * s_grid[i+1]
        if best is None or d < best['d']:
            # heading at foot
            dx = x_sp.derivative()(s_val)
            dy = y_sp.derivative()(s_val)
            psi_r = np.arctan2(dy, dx)
            # n: signed lateral
            err_x = X - foot[0]
            err_y = Y - foot[1]
            n_val = -np.sin(psi_r) * err_x + np.cos(psi_r) * err_y
            best = {'d': d, 's': s_val, 'n': n_val, 'psi_r': psi_r}

    alpha = wrap_to_pi(psi - best['psi_r'])
    return best['s'], best['n'], alpha


def frenet_to_cartesian(s, n, alpha, path_info):
    """
    (s, n, alpha) -> (X, Y, psi)
    """
    x_sp = path_info["x_spline"]
    y_sp = path_info["y_spline"]
    
    s = float(np.clip(s, 0.0, path_info["s_total"]))
    Xr = float(x_sp(s))
    Yr = float(y_sp(s))
    dx = float(x_sp.derivative()(s))
    dy = float(y_sp.derivative()(s))
    psi_r = np.arctan2(dy, dx)
    X = Xr - np.sin(psi_r) * n
    Y = Yr + np.cos(psi_r) * n
    psi = wrap_to_pi(psi_r + alpha)
    return X, Y, psi