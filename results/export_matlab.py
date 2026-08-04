#!/usr/bin/env python3
"""
Export every figure/experiment dataset (.npy/.npz) to MATLAB .mat files.

Usage:
    python3 results/export_matlab.py [--out results/matlab]

Output (default `results/matlab/`):
    traj/*.mat, ablation/*.mat, residual_data/*.mat, dbglog/*.mat,
    cloop/*.mat, waypoints/*.mat      one .mat per source file
    all_data.mat                      everything in one nested struct
    README.md                         변수/컬럼 설명 + 그림 매핑

Each per-file .mat holds:
    - one Nx1 double per column, named by physical meaning (s, n, alpha, ...)
    - `raw`  : the original N x C matrix
    - `cols` : 1xC cell of column names
    - `src`  : source path (relative to CarMaker root)
"""
import os
import sys
import numpy as np
from scipy.io import savemat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # mpc_docker/
CM = os.path.dirname(ROOT)                                            # CarMaker/
HOST = os.path.join(CM, "mpc_host")

# ---- column layouts -------------------------------------------------------
COLS_TRAJ = ["s", "n", "alpha", "v", "D", "delta"]
COLS_DBG = ["s", "n", "alpha", "v", "D", "delta", "bjac_norm",
            "res_dn", "res_dalpha", "res_dv", "tgt_delta", "status"]
COLS_X8 = ["n", "alpha", "v", "D", "delta", "kappa", "mass", "cog_x"]
COLS_X6 = ["n", "alpha", "v", "D", "delta", "kappa"]
COLS_Y = ["ds", "dn", "dalpha", "dv"]
COLS_WP = ["x", "y"]


def _fieldname(name):
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    return ("v_" + out) if not out[0].isalpha() else out


def _cols_for(arr, kind):
    c = arr.shape[1] if arr.ndim == 2 else 1
    if kind == "traj" and c == 6:
        return COLS_TRAJ
    if kind == "dbg" and c == 12:
        return COLS_DBG
    if kind == "wp" and c == 2:
        return COLS_WP
    return [f"c{i}" for i in range(c)]


def pack_matrix(arr, cols, src):
    d = {name: np.asarray(arr[:, i], dtype=np.float64).reshape(-1, 1)
         for i, name in enumerate(cols)}
    d["raw"] = np.asarray(arr, dtype=np.float64)
    d["cols"] = np.array(cols, dtype=object).reshape(1, -1)
    d["src"] = src
    return d


def pack_dataset(z, src):
    """Residual dataset .npz: X (N,6|8) features, y (N,4) residuals."""
    X, y = np.asarray(z["X"], np.float64), np.asarray(z["y"], np.float64)
    xc = COLS_X8 if X.shape[1] == 8 else (COLS_X6 if X.shape[1] == 6
                                          else [f"x{i}" for i in range(X.shape[1])])
    d = {name: X[:, i].reshape(-1, 1) for i, name in enumerate(xc)}
    d.update({name: y[:, i].reshape(-1, 1) for i, name in enumerate(COLS_Y)})
    d["X"], d["y"] = X, y
    d["X_cols"] = np.array(xc, dtype=object).reshape(1, -1)
    d["y_cols"] = np.array(COLS_Y, dtype=object).reshape(1, -1)
    d["src"] = src
    return d


# ---- what to export -------------------------------------------------------
# (group, matlab-name, absolute path, kind)
def build_jobs():
    R = os.path.join(ROOT, "results")
    jobs = []

    def add(group, name, path, kind):
        if os.path.exists(path):
            jobs.append((group, name, path, kind))
        else:
            print(f"  ! missing, skipped: {path}", file=sys.stderr)

    # 1) closed-loop trajectories (fig1-fig5, fig8)
    for tag in ("nom", "ff"):
        for m in (32, 48, 56):
            add("traj", f"{tag}_{m}", os.path.join(R, f"traj_{tag}_{m}.npy"), "traj")

    # 2) conditioning ablation (fig9)
    for tag in ("cond", "nomass"):
        for m in (32, 48, 56):
            for s in ("", "_s1", "_s2"):
                add_p = os.path.join(R, "ablation", f"abl_{tag}_{m}{s}.npy")
                if os.path.exists(add_p):
                    jobs.append(("ablation", f"{tag}_{m}{s or '_s0'}", add_p, "traj"))

    # 3) residual training / validation datasets
    for p, name in [
        (os.path.join(R, "ablation", "hockenheim_nomass.npz"), "hockenheim_nomass"),
        (os.path.join(R, "data", "cog_front48.npz"), "cog_front48"),
        (os.path.join(R, "data", "cog_rear48.npz"), "cog_rear48"),
        (os.path.join(R, "data", "hs14_32.npz"), "hs14_32"),
        (os.path.join(R, "data", "hs14_56.npz"), "hs14_56"),
        (os.path.join(ROOT, "data", "truck_dataset_combined.npz"), "truck_combined"),
        (os.path.join(HOST, "hockenheim_mass.npz"), "hockenheim_mass"),
        (os.path.join(HOST, "hockenheim_test12.npz"), "hockenheim_test12"),
        (os.path.join(HOST, "hockenheim_dataset.npz"), "hockenheim_base"),
        (os.path.join(HOST, "hockenheim_ff_check.npz"), "hockenheim_ff_check"),
        (os.path.join(HOST, "payload_dataset.npz"), "payload_dataset"),
        (os.path.join(HOST, "nomass_baseline.npz"), "nomass_baseline"),
        (os.path.join(HOST, "excite_probe8.npz"), "excite_probe8"),
        (os.path.join(HOST, "excite_probe12.npz"), "excite_probe12"),
        (os.path.join(HOST, "truck_dataset.npz"), "truck_dataset"),
        (os.path.join(HOST, "truck_dataset_exc.npz"), "truck_dataset_exc"),
        (os.path.join(HOST, "truck_dataset_renom.npz"), "truck_dataset_renom"),
    ]:
        add("residual_data", name, p, "dataset")

    # 4) per-step debug logs (figM1 / figM2 / figT7: Jacobian & steering authority)
    for f in ("ff", "js025", "js05", "js10", "jsneg05"):
        add("dbglog", f, os.path.join(R, "logs", f"{f}.npy"), "dbg")
    for f in ("dbg_jr05_48", "dbg_jr05ff_48", "dbg_jr05sqp_48",
              "dbg_js05_48", "dbg_mask03_48", "dbg_mask10_48"):
        add("dbglog", f.replace("dbg_", ""), os.path.join(HOST, f + ".npy"), "dbg")
    hl = os.path.join(HOST, "logs")
    if os.path.isdir(hl):
        for f in sorted(os.listdir(hl)):
            if f.startswith("R_") and f.endswith(".npy"):
                add("dbglog", "Rsweep_" + f[2:-4], os.path.join(hl, f), "dbg")

    # 5) first-order vs feedforward closed-loop runs (fig6)
    for f in ("cloop_first-order", "cloop_fo_d1", "cloop_fo_d4", "cloop_fo_d8",
              "cloop_fo_s1", "cloop_ff_d4", "cloop_ff_d8", "cloop_ff_s1",
              "cloop_js05_s1", "traj_jr05_48", "traj_jr05ff_48", "traj_js05_48",
              "traj_mask03_48", "traj_mask10_48", "traj_nominal_48k",
              "traj_residual_48k", "traj_nominal_10", "traj_residual_10",
              "traj_residual_10_s03", "traj_residual_10_rd"):
        add("cloop", f, os.path.join(HOST, f + ".npy"), "traj")

    # 6) reference paths
    add("waypoints", "hockenheim_1lap", os.path.join(HOST, "hockenheim_waypoints_1lap.npy"), "wp")
    add("waypoints", "hockenheim_full", os.path.join(HOST, "hockenheim_waypoints.npy"), "wp")
    add("waypoints", "ref_waypoints", os.path.join(ROOT, "data", "ref_waypoints.npy"), "wp")
    return jobs


def main():
    out = os.path.join(ROOT, "results", "matlab")
    if "--out" in sys.argv:
        out = os.path.abspath(sys.argv[sys.argv.index("--out") + 1])

    jobs = build_jobs()
    master, inv, n_ok = {}, [], 0
    for group, name, path, kind in jobs:
        rel = os.path.relpath(path, CM)
        if kind == "dataset":
            z = np.load(path, allow_pickle=True)
            d = pack_dataset(z, rel)
            shape = f"X {z['X'].shape[0]}x{z['X'].shape[1]}, y {z['y'].shape[0]}x4"
        else:
            a = np.load(path)
            if a.ndim == 1:
                a = a.reshape(-1, 1)
            d = pack_matrix(a, _cols_for(a, kind), rel)
            shape = f"{a.shape[0]}x{a.shape[1]}"
        gdir = os.path.join(out, group)
        os.makedirs(gdir, exist_ok=True)
        mpath = os.path.join(gdir, f"{name}.mat")
        savemat(mpath, d, do_compression=True)
        master.setdefault(_fieldname(group), {})[_fieldname(name)] = d
        inv.append((group, name, shape, os.path.getsize(mpath) / 1024, rel))
        n_ok += 1
        print(f"  {group}/{name}.mat  <- {rel}")

    savemat(os.path.join(out, "all_data.mat"), master,
            do_compression=True, long_field_names=True)
    write_inventory(out, inv)
    print(f"\n{n_ok} files -> {out}  (+ all_data.mat, INVENTORY.md)")


def write_inventory(out, inv):
    """Regenerate INVENTORY.md: every exported .mat with its shape and source."""
    lines = ["# 내보낸 .mat 목록 (자동 생성 — `python3 results/export_matlab.py`)", "",
             f"총 {len(inv)}개 파일. 컬럼 의미는 README.md 참고.", ""]
    for group in dict.fromkeys(r[0] for r in inv):
        rows = [r for r in inv if r[0] == group]
        total = sum(r[3] for r in rows) / 1024
        lines += [f"## `{group}/` — {len(rows)}개, {total:.1f} MB", "",
                  "| 파일 | 크기(행×열) | 원본 |", "|---|---|---|"]
        lines += [f"| `{n}.mat` | {sh} | `{src}` |" for _, n, sh, _, src in rows]
        lines.append("")
    with open(os.path.join(out, "INVENTORY.md"), "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
