%PLOT_FIG10_HELDOUT  held-out mass 실험: 무게 조건화가 미지 적재로 일반화되는가.
%   각 적재를 학습에서 완전히 제외(정규화 통계 포함)하고, 그 적재에서 잔차
%   예측 R^2 를 비교한다. 외삽(32/56 t)은 x축 라벨에 표시.
%   데이터: heldout/heldout_mass.mat  (results/heldout_mass.py 로 생성)

clear; close all;
d = load('heldout/heldout_mass.mat');

mc = mean(d.r2_dn_cond,  2);  sc = std(d.r2_dn_cond,  0, 2);
ms = mean(d.r2_dn_state, 2);  ss = std(d.r2_dn_state, 0, 2);

lbl = cell(numel(d.mass),1);
for i = 1:numel(d.mass)
    kind = 'interp';
    if d.is_extrap(i), kind = 'EXTRAP'; end
    lbl{i} = sprintf('%.0f t\n(%s)', d.mass(i)/1000, kind);
end

figure('Color', 'w', 'Position', [100 100 1000 420]);

% ---- (1) held-out 적재에서의 R^2(dn) ----
subplot(1,2,1); hold on; grid on; box on;
h = bar([mc ms]);
set(gca, 'XTick', 1:numel(d.mass), 'XTickLabel', lbl);
off = [-0.15 0.15];  V = {d.r2_dn_cond, d.r2_dn_state};
for k = 1:2
    for i = 1:numel(d.mass)
        plot(i + off(k) + zeros(1, size(V{k},2)), V{k}(i,:), 'ko', 'MarkerFaceColor', 'w');
    end
end
errorbar((1:numel(d.mass)) + off(1), mc, sc, 'k', 'LineStyle', 'none');
errorbar((1:numel(d.mass)) + off(2), ms, ss, 'k', 'LineStyle', 'none');
ylim([0.85 1.0]); ylabel('R^2 (\Deltan) on held-out mass');
legend(h, {'Mass-conditioned (8-D)', 'State-only (6-D)'}, 'Location', 'southwest');
title('Generalization to an unseen payload');

% ---- (2) in-distribution val 과의 대비 ----
subplot(1,2,2); hold on; grid on; box on;
vc = mean(d.r2val_dn_cond(:));  vs = mean(d.r2val_dn_state(:));
bar([vc vs; mean(mc) mean(ms)]);
set(gca, 'XTick', 1:2, 'XTickLabel', {sprintf('In-distribution\n(val)'), ...
                                      sprintf('Held-out mass\n(mean of 4)')});
ylim([0.85 1.0]); ylabel('R^2 (\Deltan)');
legend({'Mass-conditioned', 'State-only'}, 'Location', 'southwest');
title({'Conditioning gains nothing in-distribution,', 'and loses out-of-distribution'});

fprintf('in-distribution val : cond %.4f  state %.4f  (diff %+.4f)\n', vc, vs, vc-vs);
fprintf('held-out mass       : cond %.4f  state %.4f  (diff %+.4f)\n', ...
        mean(mc), mean(ms), mean(mc)-mean(ms));
kinds = {'interp', 'EXTRAP'};
for i = 1:numel(d.mass)
    fprintf('  %2.0f t %-7s cond %.3f+-%.3f   state %.3f+-%.3f   RMSE %+.0f%%\n', ...
            d.mass(i)/1000, kinds{d.is_extrap(i)+1}, mc(i), sc(i), ms(i), ss(i), ...
            100*(mean(d.rmse_dn_cond(i,:))/mean(d.rmse_dn_state(i,:)) - 1));
end
