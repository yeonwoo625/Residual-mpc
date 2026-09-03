% Highway L1 — 횡방향 오차 n vs 경로거리 s
%
% 참조 경로는 n=0 인 가로 직선이 된다(Frenet 좌표). 두 축이 서로 다른 물리량이라
% 눈금 간격이 달라도 왜곡이 아니며, 축 값은 실제 미터다.
% n > 0 = 진행방향 기준 왼쪽. 곡률 상위 20% 구간(급코너)은 음영으로 표시한다.
%
% 출발 직후 구간(s < 30 m)은 제외한다. L1 경로가 차량 초기 위치에서 1 m 떨어져
% 있어 |n| 이 1.0 m 로 시작하는데, 이는 제어 성능이 아니라 초기 오프셋이다.
% 과도구간은 s ~ 16 m 에서 |n| < 0.15 로 가라앉는다.
%
% 데이터: fig_highway_L1.mat  (results/highway_L1.py 가 생성)
% 요구:   base MATLAB 만. R2013a 이상.

clear; close all

% 기본 = v=20, 초기속도 72 km/h (주행의 84% 가 15 m/s 이상. 고속 검증의 정본).
%   'fig_highway_L1.mat'      v=12, 정지 출발
%   'fig_highway_L1_v20.mat'  v=20, 정지 출발 (가속 구간이 대부분이라 참고용)
MATFILE = 'fig_highway_L1_v20i.mat';
load(MATFILE)

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
REF  = [0.72 0.72 0.72];

fprintf('\n=== Highway L1, v=%.0f : lateral error (s >= %.0f m) ===\n', v_target, s_min);
fprintf('%14s %12s %12s %12s %16s\n', '', 'Y RMSE [m]', 'mean|n|', 'max|n|', 'corner RMSE');
for i = 1:numel(variant)
    fprintf('%14s %12.3f %12.3f %12.3f %16.3f\n', variant{i}, ...
            met(i,1), met(i,2), met(i,3), met(i,6));
end
fprintf('%14s %11.1f%% %11.1f%% %11.1f%% %15.1f%%\n', 'improvement', ...
        100*(met(1,1)-met(2,1))/met(1,1), 100*(met(1,2)-met(2,2))/met(1,2), ...
        100*(met(1,3)-met(2,3))/met(1,3), 100*(met(1,6)-met(2,6))/met(1,6));
fprintf('corner = top 20%% curvature (|kappa| >= %.5f, R <= %.0f m)\n\n', ...
        kappa_thr, 1/kappa_thr);

figure('Color','w','Position',[80 80 900 420]); hold on; box off
set(gca,'FontSize',11,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')

sn = traj_s{1}; nn = traj_n{1};
sr = traj_s{2}; nr = traj_n{2};
wn = sn >= s_min;  wr = sr >= s_min;
ylo = min([min(nn(wn)), min(nr(wr))]);
yhi = max([max(nn(wn)), max(nr(wr))]);
pad = (yhi-ylo)*0.22;  ylo = ylo-pad;  yhi = yhi+pad;

% 급코너 음영 - 곡률 상위 20% 구간
inC = abs(ref_kappa) >= kappa_thr;
d   = diff([0; inC(:); 0]);
b1  = find(d ==  1);   b2 = find(d == -1) - 1;
for k = 1:numel(b1)
    xa = ref_s(b1(k));  xb = ref_s(b2(k));
    if xb - xa < 5, continue; end          % 아주 짧은 구간은 생략
    patch([xa xb xb xa], [ylo ylo yhi yhi], [0.94 0.94 0.94], 'EdgeColor','none');
end

h0 = plot([s_min max(sn)], [0 0], '-', 'Color', REF, 'LineWidth', 3);
h1 = plot(sn(wn), nn(wn), '-', 'Color', NOM, 'LineWidth', 1.8);
h2 = plot(sr(wr), nr(wr), '-', 'Color', RES, 'LineWidth', 1.8);
legend([h0 h1 h2], {'reference path  (n = 0)', variant{1}, variant{2}}, ...
       'Location','southoutside','Orientation','horizontal','Box','off','FontSize',10);
text(s_min+15, ylo+(yhi-ylo)*0.08, ...
     sprintf('RMSE  %.3f  vs  %.3f m      max  %.3f  vs  %.3f m', ...
             met(1,1), met(2,1), met(1,3), met(2,3)), ...
     'FontSize',10,'Color',[0.3 0.3 0.3]);
text(max(sn)*0.97, yhi*0.86, 'shaded = top 20% curvature', ...
     'HorizontalAlignment','right','FontSize',9,'Color',[0.55 0.55 0.55]);
xlim([s_min max(sn)]); ylim([ylo yhi])
xlabel('Path distance  s  [m]'); ylabel('Lateral deviation  n  [m]')
title({'Highway No.1 (shifted path)  -  lateral error', note}, ...
      'FontSize',12,'FontWeight','normal')
