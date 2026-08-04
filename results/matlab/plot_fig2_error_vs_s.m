%PLOT_FIG2_ERROR_VS_S  경로거리 s 에 따른 횡오차 |n| (급코너 음영).
%   fig2_error_vs_s.png 재현.
%   분홍 세로띠 = 급코너 (곡률 |kappa| >= KTHR, 즉 R <= 1/KTHR).
%   통계는 lap_stats (1랩 컷 + 거리 가중) — fig1/fig3/fig9 와 동일 정의.

clear; close all;
M    = [32 48 56];
KTHR = 0.020;                     % 급코너 기준 곡률 [1/m]. R <= 1/KTHR = 50 m
T    = load_track();

% --- 급코너 구간 (모든 subplot 공통) ---
sg  = linspace(0, T.s_total, 2000)';
cm  = corner_mask(T, sg, 'kappa', KTHR);
d   = diff([0; cm; 0]);
st  = find(d > 0);  en = find(d < 0) - 1;
fprintf('급코너 = |kappa| >= %.3f (R <= %.0f m), 트랙의 %.0f%%, 구간 %d개\n\n', ...
        KTHR, 1/KTHR, 100*mean(cm), numel(st));

% --- 통계 (1랩 + 거리 가중) + 공통 y 범위 ---
Sn = cell(numel(M),1);  Sf = cell(numel(M),1);  YL = 0;
for i = 1:numel(M)
    nom = load(sprintf('traj/nom_%d.mat', M(i)));
    ff  = load(sprintf('traj/ff_%d.mat',  M(i)));
    Sn{i} = lap_stats(nom.s, nom.n, T, KTHR);
    Sf{i} = lap_stats(ff.s,  ff.n,  T, KTHR);
    YL = max([YL; Sn{i}.v; Sf{i}.v]);
    fprintf('%d t: nominal %d -> %d,  residual %d -> %d 샘플 (1랩 컷)\n', ...
            M(i), Sn{i}.n_raw, Sn{i}.n_lap, Sf{i}.n_raw, Sf{i}.n_lap);
end
YL = 1.05 * YL;

figure('Color', 'w', 'Position', [100 100 900 750]);
axs = gobjects(numel(M), 1);
for i = 1:numel(M)
    axs(i) = subplot(numel(M), 1, i); hold on; grid on; box on;
    set(gca, 'Layer', 'top');                   % grid 를 patch 위로

    hp = [];                                    % 급코너 음영 (선보다 먼저)
    for j = 1:numel(st)
        h = patch([sg(st(j)) sg(en(j)) sg(en(j)) sg(st(j))], [0 0 YL YL], ...
                  [1 .90 .90], 'EdgeColor', 'none');
        if j == 1, hp = h; else, set(h, 'HandleVisibility', 'off'); end
        if i == 1                               % 맨 위 패널에만 코너 번호
            text(mean([sg(st(j)) sg(en(j))]), 0.95*YL, sprintf('C%d', j), ...
                 'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
                 'Color', [.6 .2 .2]);
        end
    end

    hn = plot(Sn{i}.s, Sn{i}.v, 'LineWidth', 1.2, 'Color', [.85 .33 .10]);
    hf = plot(Sf{i}.s, Sf{i}.v, 'LineWidth', 1.2, 'Color', [0 .45 .74]);
    ylim([0 YL]); xlim([0 T.s_total]);
    ylabel('Lateral error  |n|  [m]');
    axtoolbar(gca, {});

    title(sprintf('%d t   Nominal %.3f m  \\rightarrow  Residual %.3f m  (%.0f%%)', ...
          M(i), Sn{i}.mean, Sf{i}.mean, 100*(Sf{i}.mean/Sn{i}.mean - 1)));

    if i < numel(M), set(gca, 'XTickLabel', []); end
    if i == 1
        legend([hp hn hf], ...
               {sprintf('Sharp corner (R \\leq %.0f m)', 1/KTHR), ...
                'Nominal MPC', 'Residual MPC'}, 'Location', 'northwest');
    end
end
xlabel('Distance along path,  s  [m]   (0 = start,  1 lap = 2546 m)');
linkaxes(axs, 'x');
