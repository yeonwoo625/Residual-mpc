% Highway L1 — Y / Yaw 오차 지표 (막대 + 표)
%
%   Y   = 횡편차 n 의 RMSE [m]        목표 n = 0 (경로 위)
%   Yaw = 헤딩오차 alpha 의 RMSE [deg] 목표 alpha = 0 (경로와 나란함)
%
% 왼쪽 칸은 절대값, 오른쪽 칸은 nominal 대비 감소율이다. 평가 구간은 s >= 30 m
% (출발 오프셋 제외). 급코너는 곡률 상위 20% 구간이다.
%
% 두 주행 모두 10 Hz 다 — 솔버 수정(SQP_ITER=10) 이후라 제어율이 맞는 비교다.
% 기존 Highway 주행은 잔차가 6.9 Hz 였다.
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
GREY = [0.55 0.55 0.55];

sel  = [1 6 4];                       % Y RMSE, corner Y RMSE, Yaw RMSE
lab  = {'Y RMSE', 'Y RMSE (corner)', 'Yaw RMSE'};
unit = {'[m]', '[m]', '[deg]'};

fprintf('\n=== Highway L1 : tracking error ===\n%s\n', note);
fprintf('[all,  s >= %.0f m]\n', s_min);
fprintf('%14s', ''); fprintf('%18s', metric{:}); fprintf('\n');
for i = 1:numel(variant)
    fprintf('%14s', variant{i}); fprintf('%18.3f', met(i,:)); fprintf('\n');
end
fprintf('%14s', 'improvement');
for j = 1:size(met,2)
    if j >= 7, fprintf('%17s ', '-');
    else       fprintf('%17.1f%%', 100*(met(1,j)-met(2,j))/met(1,j)); end
end
fprintf('\n');
if ~isnan(met_hi(1,1))
    fprintf('[high speed,  v >= %.0f m/s]\n', v_hi);
    fprintf('%14s', ''); fprintf('%18s', metric{:}); fprintf('\n');
    for i = 1:numel(variant)
        fprintf('%14s', variant{i}); fprintf('%18.3f', met_hi(i,:)); fprintf('\n');
    end
    fprintf('%14s', 'improvement');
    for j = 1:size(met_hi,2)
        if j >= 7, fprintf('%17s ', '-');
        else       fprintf('%17.1f%%', 100*(met_hi(1,j)-met_hi(2,j))/met_hi(1,j)); end
    end
    fprintf('\n');
end
fprintf('\nBoth runs at 10 Hz. Corner = top 20%% curvature (R <= %.0f m).\n\n', 1/kappa_thr);

figure('Color','w','Position',[80 80 940 400])

%% (a) 절대값 - Y 와 Yaw 는 단위가 달라 각각 정규화 없이 두 축으로 나눈다
subplot(1,2,1); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
x = (1:3)'; w = 0.34;
b1 = bar(x-w/2, met(1,sel)', w, 'FaceColor', NOM, 'EdgeColor','none');
b2 = bar(x+w/2, met(2,sel)', w, 'FaceColor', RES, 'EdgeColor','none');
for k = 1:3
    text(x(k)-w/2, met(1,sel(k))*1.06, sprintf('%.3f', met(1,sel(k))), ...
         'HorizontalAlignment','center','FontSize',9,'Color',NOM);
    text(x(k)+w/2, met(2,sel(k))*1.06 + max(met(1,sel))*0.02, ...
         sprintf('%.3f', met(2,sel(k))), ...
         'HorizontalAlignment','center','FontSize',9,'Color',RES);
end
legend([b1 b2], variant, 'Location','northwest','Box','off','FontSize',10);
xt = cell(1,3);
for k = 1:3
    xt{k} = sprintf('%s %s', lab{k}, unit{k});
end
set(gca,'XTick',1:3,'XTickLabel', xt)
xlim([0.4 3.6]); ylim([0 max(met(1,sel))*1.25])
ylabel('RMSE   (m for Y,  deg for Yaw)')
title({'(a)  Absolute error', note}, 'FontSize',12,'FontWeight','normal')

%% (b) 감소율
subplot(1,2,2); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
red = 100*(met(1,sel)-met(2,sel))./met(1,sel);
bar(x, red(:), 0.55, 'FaceColor', RES, 'EdgeColor','none');
plot([0.4 3.6],[0 0],'-','Color',[0.3 0.3 0.3],'LineWidth',0.9);
for k = 1:3
    text(x(k), red(k)+3, sprintf('%.1f%%', red(k)), ...
         'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
end
set(gca,'XTick',1:3,'XTickLabel', lab)
xlim([0.4 3.6]); ylim([0 100])
ylabel('Reduction vs nominal  [%]')
title('(b)  Improvement','FontSize',12,'FontWeight','normal')
