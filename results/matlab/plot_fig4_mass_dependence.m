%PLOT_FIG4_MASS_DEPENDENCE  적재 -> 횡잔차 |dn| (조건화 동기).
%   fig4_mass_dependence.png 재현. 데이터: residual_data/hockenheim_mass.mat
%   (X = [n,alpha,v,D,delta,kappa,mass,cog_x],  y = [ds,dn,dalpha,dv])

clear; close all;
d  = load('residual_data/hockenheim_mass.mat');
cm = abs(d.kappa) >= pctl(abs(d.kappa), 0.8);          % 급코너 상위 20% (pctl.m)

mv = unique(d.mass);
mu_all = zeros(size(mv));  mu_cor = zeros(size(mv));  se_cor = zeros(size(mv));
for i = 1:numel(mv)
    ia = d.mass == mv(i);          ic = ia & cm;
    mu_all(i) = mean(abs(d.dn(ia)));
    mu_cor(i) = mean(abs(d.dn(ic)));
    se_cor(i) = std(abs(d.dn(ic))) / sqrt(sum(ic));
end

R = corrcoef(mv, mu_cor);                                % 툴박스 불필요
fprintf('corr(mass, corner mean |dn|) = %+.3f\n', R(1,2));

figure('Color', 'w', 'Position', [100 100 950 400]);

subplot(1,2,1); hold on; grid on; box on;
errorbar(mv/1000, mu_cor, se_cor, 'o-', 'LineWidth', 1.5, 'MarkerFaceColor', 'w');
plot(mv/1000, mu_all, 's--', 'LineWidth', 1.2);
xlabel('총 적재질량 [t]'); ylabel('mean |\Deltan| [m/step]');
legend({'급코너', '전체'}, 'Location', 'northwest');
title(sprintf('적재 -> 횡잔차   corr = %+.2f', R(1,2)));

subplot(1,2,2); hold on; grid on; box on;
for i = 1:numel(mv)
    ic = (d.mass == mv(i)) & cm;
    plot(d.mass(ic)/1000 + 0.6*(rand(sum(ic),1)-0.5), d.dn(ic), '.', 'MarkerSize', 4);
end
xlabel('총 적재질량 [t]'); ylabel('\Deltan [m/step]');
title('급코너 잔차 분포 (jitter)');
