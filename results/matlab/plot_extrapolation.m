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

figure('Color','w','Position',[80 80 1000 720])

%% (a) 속도별 급코너 오차 - 학습 상한 밖에서도 유지되는가
subplot(2,2,1); hold on; box off
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
subplot(2,2,2); hold on; box off
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

%% (c) 세 지표가 함께 줄었는가 - 횡오차 / 헤딩오차 / 조향 채터
%     n 만 보고하면 "유리한 지표만 골랐다"는 반박을 받는다. alpha 는 v=10 에서
%     오히려 나빠지는데(횡오차를 줄이려 차체를 더 돌리는 trade-off), 외삽
%     조건인 v=14 에서는 세 지표가 모두 개선된다.
subplot(2,2,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
% 3번째 색 = Okabe-Ito bluish green. 파랑/주황과 함께 적록색약에서도 구분된다
% (기존 초록/빨강 조합이 D형에서 안 구분돼 저장소 전체가 파랑/주황으로 바뀐 이력).
HEAD = [0.00 0.62 0.45];
red_n = speed.improve(:);
red_a = speed.a_improve(:);
red_c = 100*(1 - speed.chat_res(:)./speed.chat_nom(:));
w = 0.26; x = (1:numel(vt))';
b1 = bar(x-w,   red_n, w, 'FaceColor', RES,  'EdgeColor','none');
b2 = bar(x,     red_a, w, 'FaceColor', HEAD, 'EdgeColor','none');
b3 = bar(x+w,   red_c, w, 'FaceColor', NOM,  'EdgeColor','none');
plot([0.4 numel(vt)+0.6], [0 0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9);
% 마지막(외삽) 조건 강조
plot([numel(vt)-0.5 numel(vt)-0.5], [-300 100], ':', 'Color', GREY, 'LineWidth', 1.4);
text(numel(vt), 88, 'extrapolation', 'HorizontalAlignment','center', ...
     'FontSize',9,'Color',GREY);
legend([b1 b2 b3], {'Lateral error  n','Heading error  \alpha','Steering chatter'}, ...
       'Location','southwest', 'Box','off', 'FontSize', 9);
set(gca,'XTick',1:numel(vt),'XTickLabel', ...
        arrayfun(@(v) sprintf('%.1f',v), vt, 'UniformOutput', false))
% v=10 채터는 -265% (잔차가 채터를 4배 늘린다). 잘리지 않게 축을 -300 까지 둔다.
xlim([0.4 numel(vt)+0.6]); ylim([-300 100])
xlabel('Target speed  [m/s]'); ylabel('Reduction vs nominal  [%]')
title('(c)  Lateral, heading, chatter','FontSize',11,'FontWeight','normal')

%% (d) 대가 - 종방향(x) 진행오차와 랩타임
%     y/yaw 가 좋아지는 대신 잔차는 목표 속도를 덜 지킨다. v=14 에서 573 m
%     뒤처지고 랩타임이 29 s 길다. 이 비용을 숨기지 않고 같이 싣는다.
subplot(2,2,4); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
xn = speed.x_nom(:);  xr = speed.x_res(:);
w2 = 0.34;
d1 = bar(x-w2/2, xn, w2, 'FaceColor', NOM, 'EdgeColor','none');
d2 = bar(x+w2/2, xr, w2, 'FaceColor', RES, 'EdgeColor','none');
plot([0.4 numel(vt)+0.6], [0 0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9);
plot([numel(vt)-0.5 numel(vt)-0.5], [-650 120], ':', 'Color', GREY, 'LineWidth', 1.4);
text(numel(vt), 80, 'extrapolation', 'HorizontalAlignment','center', ...
     'FontSize',9,'Color',GREY);
% 랩타임 차이를 숫자로
for k = 1:numel(vt)
    text(x(k), -620, sprintf('%+.0f s', speed.lap_res(k)-speed.lap_nom(k)), ...
         'HorizontalAlignment','center','FontSize',9,'Color',[0.35 0.35 0.35]);
end
legend([d1 d2], {'Nominal MPC','Residual MPC'}, 'Location','southwest', ...
       'Box','off', 'FontSize', 9);
set(gca,'XTick',1:numel(vt),'XTickLabel', ...
        arrayfun(@(v) sprintf('%.1f',v), vt, 'UniformOutput', false))
xlim([0.4 numel(vt)+0.6]); ylim([-650 120])
xlabel('Target speed  [m/s]   (label = lap time vs nominal)')
ylabel('Longitudinal error  x  [m]')
title('(d)  The cost: falls behind schedule','FontSize',11,'FontWeight','normal')
