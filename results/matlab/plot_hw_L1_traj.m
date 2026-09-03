% Highway L1, v=12 — 주행 궤적 (전역 좌표)
%
%   왼쪽 칸  전체 경로 (932 m). 축 비율 1:1. 세 선이 거의 겹친다 - 사실이다.
%            932 m 경로에서 최대 편차가 0.84 m 라 화면상 0.1% 다.
%   오른쪽 칸 편차가 가장 큰 지점(s ~ 524 m) 확대. 반폭 ZOOM_HALF.
%
% 과장 배율은 쓰지 않는다. 지도는 X, Y 가 같은 물리량이라 axis equal 이 필수이고,
% 편차만 K 배 늘리면 한 그림에 두 배율이 섞여 축 눈금을 정직하게 붙일 수 없다.
% 차이를 정량적으로 보려면 plot_hw_L1_lateral (경로 좌표) 을 쓴다.
%
% 데이터: fig_highway_L1.mat  (results/highway_L1.py 가 생성)
% 요구:   base MATLAB 만. R2013a 이상.

clear; close all
load('fig_highway_L1.mat')

ZOOM_HALF = 12;        % 확대 창 반폭 [m]. 줄이면 편차가 더 크게 보인다

NOM = [0.12 0.47 0.71];
RES = [0.90 0.49 0.13];
REF = [0.72 0.72 0.72];

sn = traj_s{1};  xn = traj_x{1};  yn = traj_y{1};  nn = traj_n{1};
sr = traj_s{2};  xr = traj_x{2};  yr = traj_y{2};

% 편차가 가장 큰 지점 (출발 오프셋 제외)
w = sn >= s_min;
idx = find(w);
[~, k] = max(abs(nn(w)));
sz = sn(idx(k));
cx = interp1(ref_s, ref_x, sz);
cy = interp1(ref_s, ref_y, sz);

fprintf('\n=== Highway L1, v=12 : trajectory ===\n');
fprintf('path length %.0f m, min radius %.1f m\n', s_total, 1/max(abs(ref_kappa)));
fprintf('%14s %12s %12s %12s\n','','distance [m]','max|n| [m]','rate [Hz]');
for i = 1:numel(variant)
    fprintf('%14s %12.0f %12.3f %12.1f\n', variant{i}, met(i,8), met(i,3), met(i,7));
end
fprintf('zoom window centred at s = %.0f m (largest deviation)\n\n', sz);

figure('Color','w','Position',[60 60 1120 520])

%% (좌) 전체 경로
subplot(1,2,1); hold on; box on
set(gca,'FontSize',10,'TickDir','out')
p0 = plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 4);
p1 = plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 1.2);
p2 = plot(xr, yr, '-', 'Color', RES, 'LineWidth', 1.2);
plot([cx-ZOOM_HALF cx+ZOOM_HALF cx+ZOOM_HALF cx-ZOOM_HALF cx-ZOOM_HALF], ...
     [cy-ZOOM_HALF cy-ZOOM_HALF cy+ZOOM_HALF cy+ZOOM_HALF cy-ZOOM_HALF], ...
     '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.2);
axis equal; axis tight
legend([p0 p1 p2], {'reference path', variant{1}, variant{2}}, ...
       'Location','southoutside','Orientation','horizontal','Box','off','FontSize',9);
xlabel('X  [m]'); ylabel('Y  [m]')
title(sprintf('Full path,  %.0f m  (deviations invisible at this scale)', s_total), ...
      'FontSize',11,'FontWeight','normal')

%% (우) 최대 편차 지점 확대
subplot(1,2,2); hold on; box on
set(gca,'FontSize',10,'TickDir','out')
plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 5);
plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 2);
plot(xr, yr, '-', 'Color', RES, 'LineWidth', 2);
axis equal
xlim([cx-ZOOM_HALF cx+ZOOM_HALF]); ylim([cy-ZOOM_HALF cy+ZOOM_HALF]);
xlabel('X  [m]'); ylabel('Y  [m]')
title(sprintf('Zoom at s = %.0f m   (|n| %.3f vs %.3f m)', sz, met(1,3), met(2,3)), ...
      'FontSize',11,'FontWeight','normal')
