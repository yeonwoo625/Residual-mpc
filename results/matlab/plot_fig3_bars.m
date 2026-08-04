%PLOT_FIG3_BARS  적재별 평균 / RMS / 급코너 RMS 막대 + 요약표.
%   fig3_bar_comparison.png, fig8_multiload.png 재현.
%   통계는 lap_stats (1랩 컷 + 거리 가중) — fig1/fig2/fig9 와 동일 정의.

clear; close all;
M    = [32 48 56];
KTHR = 0.020;               % 급코너 기준 곡률 [1/m], R <= 50 m
T    = load_track();

metrics = {'mean |n|', 'RMS |n|', sprintf('corner RMS (R \\leq %.0f m)', 1/KTHR)};
V = zeros(numel(M), 2, 3);              % (적재, 방법, 지표)

fprintf('1랩 + 거리 가중,  급코너 = |kappa| >= %.3f (R <= %.0f m)\n\n', KTHR, 1/KTHR);
fprintf('%5s | %7s %7s %8s | %7s %7s %8s | %6s\n', 'mass', ...
        'nom', 'nomRMS', 'nomCorn', 'res', 'resRMS', 'resCorn', 'mean%');
for i = 1:numel(M)
    nom = load(sprintf('traj/nom_%d.mat', M(i)));
    ff  = load(sprintf('traj/ff_%d.mat',  M(i)));
    S   = {lap_stats(nom.s, nom.n, T, KTHR), lap_stats(ff.s, ff.n, T, KTHR)};
    for k = 1:2
        V(i,k,:) = [S{k}.mean, S{k}.rms, S{k}.corner_rms];
    end
    fprintf('%4dt | %7.3f %7.3f %8.3f | %7.3f %7.3f %8.3f | %5.0f%%\n', M(i), ...
            V(i,1,1), V(i,1,2), V(i,1,3), V(i,2,1), V(i,2,2), V(i,2,3), ...
            100*(V(i,2,1)/V(i,1,1) - 1));
end

figure('Color', 'w', 'Position', [100 100 1100 380]);
for m = 1:3
    subplot(1, 3, m);
    bar(V(:,:,m));
    set(gca, 'XTickLabel', {'32 t', '48 t', '56 t'});
    ylabel('Lateral error [m]'); title(metrics{m}); grid on; box on;
    if m == 1, legend({'Nominal MPC', 'Residual MPC'}, 'Location', 'northeast'); end
end
