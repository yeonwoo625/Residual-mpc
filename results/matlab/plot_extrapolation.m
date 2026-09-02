% plot_extrapolation.m — 학습 분포 밖(외삽)에서의 잔차 보정
%
% (a) 속도 스윕: 급코너 오차 nominal vs residual, 학습 상한 표시
% (b) v=14 의 속도 분포가 학습 데이터를 넘어선 정도
% (c) 개선율과 채터 감소
%
% 데이터: fig_extrapolation.mat  (results/extrapolation_summary.py 로 생성)

clear; close all;
load('fig_extrapolation.mat');

BLUE = [0.12 0.47 0.71];
ORNG = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];
SHADE= [0.93 0.93 0.93];

vt   = speed.v_target(:);
vmax = train_v_max;

figure('Color','w','Position',[100 100 1240 360]);

% ------------------------------------------------- (a) 속도별 급코너 오차
subplot(1,3,1); hold on; box on;
yl = [0 2.5];
% 학습 상한 밖 영역 음영
patch([vmax 15 15 vmax], [yl(1) yl(1) yl(2) yl(2)], SHADE, ...
      'EdgeColor','none');
plot([vmax vmax], yl, '--', 'Color', GREY, 'LineWidth', 1.4);
text(vmax+0.12, yl(2)*0.94, sprintf('학습 상한 %.1f m/s', vmax), ...
     'Color', GREY, 'FontSize', 9);
text(14, yl(2)*0.06, '외삽', 'Color', GREY, 'FontSize', 10, ...
     'HorizontalAlignment','center');
h1 = plot(vt, speed.nom(:), '-o', 'Color', BLUE, 'LineWidth', 1.8, ...
          'MarkerSize', 6, 'MarkerFaceColor', BLUE);
h2 = plot(vt, speed.res(:), '-s', 'Color', ORNG, 'LineWidth', 1.8, ...
          'MarkerSize', 6, 'MarkerFaceColor', ORNG);
for k = 1:numel(vt)
    text(vt(k), speed.res(k)-0.16, sprintf('%.0f%%', speed.improve(k)), ...
         'Color', ORNG, 'FontSize', 9, 'HorizontalAlignment','center');
end
legend([h1 h2], {'nominal','residual'}, 'Location','NorthWest', 'FontSize', 9);
legend boxoff;
set(gca,'FontSize',10,'XGrid','on','YGrid','on');
xlim([9.4 15]); ylim(yl);
xlabel('목표 속도 (m/s)'); ylabel('급코너 횡오차 RMS (m)');
title('(a) 학습 상한을 넘어서도 유지', 'FontSize',11);

% ------------------------------------------------- (b) 속도 분포
subplot(1,3,2); hold on; box on;
c = hist.centers(:);
yl2 = [0 max(hist.train)*1.15];
patch([vmax 15 15 vmax], [yl2(1) yl2(1) yl2(2) yl2(2)], SHADE, 'EdgeColor','none');
ht = plot(c, hist.train(:),   '-', 'Color', GREY, 'LineWidth', 2.0);
hr = plot(c, hist.v14_res(:), '-', 'Color', ORNG, 'LineWidth', 1.8);
hn = plot(c, hist.v14_nom(:), '-', 'Color', BLUE, 'LineWidth', 1.4);
plot([vmax vmax], yl2, '--', 'Color', GREY, 'LineWidth', 1.4);
legend([ht hn hr], {'학습 데이터','v=14 nominal','v=14 residual'}, ...
       'Location','NorthWest', 'FontSize', 9);
legend boxoff;
set(gca,'FontSize',10,'XGrid','on','YGrid','on');
xlim([8 15]); ylim(yl2);
xlabel('속도 (m/s)'); ylabel('밀도');
title('(b) v=14 는 학습 범위 밖에서 주행', 'FontSize',11);

% ------------------------------------------------- (c) 개선율 / 채터
subplot(1,3,3); hold on; box on;
chat_red = 100*(1 - speed.chat_res(:)./speed.chat_nom(:));
w = 0.34; x = (1:numel(vt))';
bar(x-w/2, speed.improve(:), w, 'FaceColor', ORNG, 'EdgeColor','none');
bar(x+w/2, chat_red,         w, 'FaceColor', BLUE, 'EdgeColor','none');
plot([0.4 numel(vt)+0.6], [0 0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.8);
for k = 1:numel(vt)
    text(x(k)-w/2, speed.improve(k)+4, sprintf('%.0f', speed.improve(k)), ...
         'HorizontalAlignment','center','FontSize',9,'Color',ORNG);
end
% 외삽 조건 표시
plot(numel(vt), -95, '^', 'Color', GREY, 'MarkerSize', 7, ...
     'MarkerFaceColor', GREY, 'Clipping','off');
text(numel(vt), -112, '외삽', 'HorizontalAlignment','center', ...
     'FontSize',9,'Color',GREY);
legend({'급코너 오차 감소','조향 채터 감소'}, 'Location','SouthWest', 'FontSize', 9);
legend boxoff;
set(gca,'XTick',1:numel(vt), 'XTickLabel', ...
        arrayfun(@(v) sprintf('%.1f',v), vt, 'UniformOutput', false), ...
        'FontSize',10,'YGrid','on');
xlim([0.4 numel(vt)+0.6]); ylim([-100 100]);
xlabel('목표 속도 (m/s)'); ylabel('nominal 대비 감소율 (%)');
title('(c) 오차와 채터 동시 감소', 'FontSize',11);

set(gcf,'PaperPositionMode','auto');
print(gcf, '-dpng', '-r200', 'fig_extrapolation.png');
fprintf('saved fig_extrapolation.png\n');
