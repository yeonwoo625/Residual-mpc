%PLOT_FIG1_TRACK  트랙 위 주행 궤적을 |n| 으로 색칠 (nominal vs residual).
%   fig1_trajectory_error.png 재현.
%   통계는 lap_stats (1랩 컷 + 거리 가중) — fig2/fig3/fig9 와 동일 정의.

clear; close all;
MASS = [32 48 56];          % [32 48 56] 세 적재 / 48 하나만도 가능
KTHR = 0.020;               % 급코너 기준 곡률 [1/m], R <= 50 m
T    = load_track();

nom = cell(size(MASS));  ff = cell(size(MASS));
Sn  = cell(size(MASS));  Sf = cell(size(MASS));  cmax = 0;
for i = 1:numel(MASS)
    nom{i} = load(sprintf('traj/nom_%d.mat', MASS(i)));
    ff{i}  = load(sprintf('traj/ff_%d.mat',  MASS(i)));
    Sn{i}  = lap_stats(nom{i}.s, nom{i}.n, T, KTHR);
    Sf{i}  = lap_stats(ff{i}.s,  ff{i}.n,  T, KTHR);
    cmax   = max([cmax; Sn{i}.v; Sf{i}.v]);        % 전 패널 공통 색범위
end

nc = numel(MASS);
figure('Color', 'w', 'Position', [80 80 min(420*nc, 1400) 760]);
for i = 1:nc
    runs = {'Nominal MPC', nom{i}, Sn{i}; 'Residual MPC', ff{i}, Sf{i}};
    for k = 1:2
        subplot(2, nc, (k-1)*nc + i); hold on;
        r = runs{k,2};  S = runs{k,3};
        kk = first_lap(r.s, T.s_total);                    % 1랩만 그린다
        [X, Y] = frenet_to_xy(T, r.s(kk), r.n(kk));
        plot(T.x, T.y, '-', 'Color', [.8 .8 .8], 'LineWidth', 3);
        scatter(X, Y, 10, abs(r.n(kk)), 'filled');
        axis equal; grid on; box on;
        caxis([0 cmax]); colormap(jet);
        c = colorbar; ylabel(c, '|n| [m]');
        title(sprintf('%s  %d t\nmean |n| = %.3f m', runs{k,1}, MASS(i), S.mean));
        xlabel('X [m]'); ylabel('Y [m]');
    end
end
