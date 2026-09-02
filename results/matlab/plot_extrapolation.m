% 외삽 일반화 - 학습 속도 상한 밖에서도 잔차 보정이 작동하는가
%
% 기존 일반화 주장(적재, 미학습 트랙)은 전부 학습 분포 '안쪽'이었다.
% 학습 데이터의 속도 최대값은 12.234 m/s 이고, v=14 주행은 14.38 m/s 까지
% 올라가 상한을 17.5% 초과한다. 그 조건에서도 잔차가 이긴다.
%   급코너 RMS  nominal 2.138 -> residual 1.211  (43.4% 감소)
%   조향 채터   0.0620 -> 0.0382 (38% 감소), 양쪽 완주
%
% 데이터: fig_extrapolation.mat  (results/extrapolation_summary.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_extrapolation`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_extrapolation.mat')

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];
SHAD = [0.94 0.94 0.94];

vt   = speed.v_target(:);
vmax = train_v_max;

figure('Color','w','Position',[80 80 1280 380])

%% (a) 속도별 급코너 오차 - 학습 상한 밖에서도 유지되는가
subplot(1,3,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
YL = [0 2.6];
patch([vmax 15.2 15.2 vmax], [YL(1) YL(1) YL(2) YL(2)], SHAD, 'EdgeColor','none');
plot([vmax vmax], YL, '--', 'Color', GREY, 'LineWidth', 1.6);
text(vmax-0.15, YL(2)*0.96, sprintf('training limit  %.2f m/s', vmax), ...
     'Color', GREY, 'FontSize', 9, 'HorizontalAlignment','right');
text(13.9, YL(2)*0.06, 'extrapolation', 'Color', GREY, 'FontSize', 10, ...
     'HorizontalAlignment','center');
h1 = plot(vt, speed.nom(:), '-o', 'Color', NOM, 'LineWidth', 2, ...
          'MarkerSize', 7, 'MarkerFaceColor', NOM);
h2 = plot(vt, speed.res(:), '-s', 'Color', RES, 'LineWidth', 2, ...
          'MarkerSize', 7, 'MarkerFaceColor', RES);
for k = 1:numel(vt)
    text(vt(k), speed.res(k)-0.19, sprintf('-%.0f%%', speed.improve(k)), ...
         'Color', RES, 'FontSize', 9, 'HorizontalAlignment','center');
end
legend([h1 h2], {'Nominal MPC','Residual MPC'}, 'Location','northwest', ...
       'Box','off', 'FontSize', 9);
xlim([9.4 15.2]); ylim(YL)
xlabel('Target speed  [m/s]'); ylabel('Corner RMS lateral error  [m]')
title('(a)  Holds beyond the training limit','FontSize',11,'FontWeight','normal')

%% (b) 속도 분포 - v=14 가 실제로 학습 범위를 벗어났음을 보인다
subplot(1,3,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
c  = vdist.centers(:);
Y2 = [0 max(vdist.train)*1.12];
patch([vmax 15.2 15.2 vmax], [Y2(1) Y2(1) Y2(2) Y2(2)], SHAD, 'EdgeColor','none');
g1 = plot(c, vdist.train(:),   '-', 'Color', GREY, 'LineWidth', 2.4);
g2 = plot(c, vdist.v14_nom(:), '-', 'Color', NOM,  'LineWidth', 1.6);
g3 = plot(c, vdist.v14_res(:), '-', 'Color', RES,  'LineWidth', 2.0);
plot([vmax vmax], Y2, '--', 'Color', GREY, 'LineWidth', 1.6);
text(vmax-0.15, Y2(2)*0.96, sprintf('training limit  %.2f m/s', vmax), ...
     'Color', GREY, 'FontSize', 9, 'HorizontalAlignment','right');
legend([g1 g2 g3], {'training data','v=14  Nominal','v=14  Residual'}, ...
       'Location','northwest', 'Box','off', 'FontSize', 9);
xlim([8 15.2]); ylim(Y2)
xlabel('Speed  [m/s]'); ylabel('Density')
title('(b)  v=14 runs outside the training range','FontSize',11,'FontWeight','normal')

%% (c) 오차와 채터가 함께 줄었는가
subplot(1,3,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
chat = 100*(1 - speed.chat_res(:)./speed.chat_nom(:));
w = 0.34; x = (1:numel(vt))';
b1 = bar(x-w/2, speed.improve(:), w, 'FaceColor', RES, 'EdgeColor','none');
b2 = bar(x+w/2, chat,             w, 'FaceColor', NOM, 'EdgeColor','none');
plot([0.4 numel(vt)+0.6], [0 0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.8);
for k = 1:numel(vt)
    text(x(k)-w/2, speed.improve(k)+5, sprintf('%.0f', speed.improve(k)), ...
         'HorizontalAlignment','center','FontSize',9,'Color',RES);
end
% 마지막 막대 = 외삽 조건
xl = get(gca,'XLim');
text(numel(vt), 92, 'extrapolation', 'HorizontalAlignment','center', ...
     'FontSize',9,'Color',GREY);
legend([b1 b2], {'Corner error reduction','Steering chatter reduction'}, ...
       'Location','southwest', 'Box','off', 'FontSize', 9);
set(gca,'XTick',1:numel(vt),'XTickLabel', ...
        arrayfun(@(v) sprintf('%.1f',v), vt, 'UniformOutput', false))
xlim([0.4 numel(vt)+0.6]); ylim([-100 100])
xlabel('Target speed  [m/s]'); ylabel('Reduction vs nominal  [%]')
title('(c)  Error and chatter both drop','FontSize',11,'FontWeight','normal')

