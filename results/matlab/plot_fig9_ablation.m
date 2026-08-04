%PLOT_FIG9_ABLATION  무게 조건화 O/X 비교 (급코너 RMS, 3시드 paired).
%   fig9_conditioning_ablation.png 재현.
%   통계는 lap_stats (1랩 컷 + 거리 가중) — fig1/fig2/fig3 와 동일 정의.

clear; close all;
M    = [32 56];
S    = {'_s0', '_s1', '_s2'};
KTHR = 0.020;               % 급코너 기준 곡률 [1/m], R <= 50 m
T    = load_track();

R = nan(numel(M), 2, numel(S));                 % (적재, cond/nomass, 시드)
for i = 1:numel(M)
    for k = 1:2
        tags = {'cond', 'nomass'};
        for j = 1:numel(S)
            f = sprintf('ablation/%s_%d%s.mat', tags{k}, M(i), S{j});
            if ~exist(f, 'file'), continue; end
            r  = load(f);
            st = lap_stats(r.s, r.n, T, KTHR);
            R(i,k,j) = st.corner_rms;
        end
    end
end
if any(isnan(R(:)))
    warning('ablation/*.mat 일부 누락 — 해당 조합은 NaN');
end

mu = mean(R, 3);  sd = std(R, 0, 3);
figure('Color', 'w', 'Position', [100 100 560 430]); hold on; grid on; box on;
h = bar(mu);  set(gca, 'XTick', 1:numel(M), 'XTickLabel', {'32 t', '56 t'});

off = [-0.15 0.15];                             % 개별 시드 점 (paired 이므로 필수)
for i = 1:numel(M)
    for k = 1:2
        v = squeeze(R(i,k,:));
        plot(i + off(k) + zeros(size(v)), v, 'ko', 'MarkerFaceColor', 'w');
        errorbar(i + off(k), mu(i,k), sd(i,k), 'k', 'LineWidth', 1.2);
    end
end
ylabel(sprintf('Corner RMS |n| [m]   (R \\leq %.0f m)', 1/KTHR));
legend(h, {'Mass-conditioned', 'State-only'}, 'Location', 'northwest');
title('Payload conditioning ablation (dots = individual seeds)');

dif = squeeze(R(:,2,:) - R(:,1,:));  dif = dif(~isnan(dif));
fprintf('paired 차이 (state-only - conditioned): mean %+.3f m, SE %.3f, n=%d\n', ...
        mean(dif), std(dif)/sqrt(numel(dif)), numel(dif));
