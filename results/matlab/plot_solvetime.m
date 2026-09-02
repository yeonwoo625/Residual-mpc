% plot_solvetime.m — MPC 연산시간: nominal vs residual, 그리고 100배 낭비의 제거
%
% (a) 주행 중 계측: residual 이 10 Hz 예산을 넘긴다
% (b) 원인: SQP 반복 횟수에 정비례 — 계산이 무거운 게 아니라 100번 돌고 있었다
% (c) 10회면 100회와 같은 해 -> 90회는 순수 낭비
%
% 데이터: fig_solvetime.mat  (results/solvetime_summary.py 로 생성)

clear; close all;
load('fig_solvetime.mat');

BLUE = [0.12 0.47 0.71];
ORNG = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];

figure('Color','w','Position',[100 100 1240 360]);

% ---------------------------------------------------------------- (a) 주행 중 계측
% 막대 = 중앙값, 수염 = p95. 3개 구성 전부 실제 주행 로그.
subplot(1,3,1); hold on; box on;
med = inloop_stats(:,1);  p95 = inloop_stats(:,3);
cols = [BLUE; ORNG; ORNG];
for k = 1:3
    bar(k, med(k), 0.55, 'FaceColor', cols(k,:), 'EdgeColor','none');
    plot([k k], [med(k) p95(k)], '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.1);
    plot(k, p95(k), '_', 'Color', [0.25 0.25 0.25], 'MarkerSize', 9, 'LineWidth', 1.1);
    text(k, p95(k)*1.18, sprintf('%.1f', med(k)), ...
         'HorizontalAlignment','center', 'FontSize', 10, 'FontWeight','bold');
end
plot([0.4 3.6], [budget_ms budget_ms], '--', 'Color', GREY, 'LineWidth', 1.4);
text(3.55, budget_ms*1.30, '10 Hz 예산 100 ms', ...
     'Color', GREY, 'FontSize', 9, 'HorizontalAlignment','right');
set(gca,'XTick',1:3,'XTickLabel',{'nominal','residual','residual'}, ...
        'YScale','log','FontSize',10,'YGrid','on');
text(2, 0.55, '수정 전', 'HorizontalAlignment','center','FontSize',9,'Color',ORNG);
text(3, 0.55, '수정 후', 'HorizontalAlignment','center','FontSize',9,'Color',ORNG);
xlim([0.4 3.6]); ylim([0.4 400]);
ylabel('solve 시간 (ms) — 막대 중앙값, 수염 p95');
title('(a) 주행 중 계측', 'FontSize',11);

% ------------------------------------------------- (b) 반복 횟수 대 solve 시간
subplot(1,3,2); hold on; box on;
it = sweep(:,1); ms = sweep(:,2);
plot([1 100], [budget_ms budget_ms], '--', 'Color', GREY, 'LineWidth', 1.4);
plot([1 100], [nominal_offline_ms nominal_offline_ms], ':', 'Color', BLUE, 'LineWidth', 1.6);
plot(it, ms, '-o', 'Color', ORNG, 'LineWidth', 1.8, 'MarkerSize', 6, ...
     'MarkerFaceColor', ORNG);
plot(10,  ms(it==10),  'o', 'Color','k', 'MarkerSize', 11, 'LineWidth', 1.6);
plot(100, ms(it==100), 'o', 'Color','k', 'MarkerSize', 11, 'LineWidth', 1.6);
text(10,  ms(it==10)*0.45, '수정 후', 'FontSize',9, 'HorizontalAlignment','center');
text(100, ms(it==100)*1.9, '수정 전', 'FontSize',9, 'HorizontalAlignment','right');
text(1.15, nominal_offline_ms*0.62, 'nominal', 'Color', BLUE, 'FontSize',9);
set(gca,'XScale','log','YScale','log','FontSize',10,'XGrid','on','YGrid','on');
xlim([0.85 130]); ylim([0.3 400]);
xlabel('제어 1스텝당 SQP 반복 횟수');
ylabel('solve 시간 (ms)');
title('(b) 시간은 반복 횟수에 정비례', 'FontSize',11);

% --------------------------------------------- (c) 해는 10회에서 이미 수렴
subplot(1,3,3); hold on; box on;
dev = sweep(:,3);                          % max |d delta| vs 100회 해
dev(dev <= 0) = 1e-4;                      % log 축 표시용 하한
plot(it, dev, '-s', 'Color', ORNG, 'LineWidth', 1.8, 'MarkerSize', 6, ...
     'MarkerFaceColor', ORNG);
plot(10, dev(it==10), 'o', 'Color','k', 'MarkerSize', 11, 'LineWidth', 1.6);
text(10, dev(it==10)*7, sprintf('%.4f\\circ', sweep(it==10,3)), ...
     'FontSize',9, 'HorizontalAlignment','center');
set(gca,'XScale','log','YScale','log','FontSize',10,'XGrid','on','YGrid','on');
xlim([0.85 130]); ylim([5e-5 40]);
xlabel('제어 1스텝당 SQP 반복 횟수');
ylabel('100회 해 대비 조향 차이 (deg)');
title('(c) 10회에서 이미 수렴', 'FontSize',11);

set(gcf,'PaperPositionMode','auto');
print(gcf, '-dpng', '-r200', 'fig_solvetime.png');
fprintf('saved fig_solvetime.png\n');
