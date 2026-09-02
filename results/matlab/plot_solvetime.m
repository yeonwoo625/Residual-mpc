% MPC 연산시간 - nominal vs residual, 그리고 100배 낭비의 제거
%
% l4acados 의 solve() 는 nlp_solver_max_iter 만큼 preparation/feedback 을 무조건
% 반복한다(수렴 시 조기 종료 분기가 rti_log_residuals 에만 있다). acados 기본값
% 100 이 그대로 적용되어 제어 1스텝마다 SQP 를 100회 돌고 있었다 - nominal 은
% acados 자체 SQP_RTI 로 1회만 돈다. 10회면 100회와 조향 명령이 0.002도 이내로
% 같으므로 나머지 90회는 수렴한 문제를 다시 푸는 순수 낭비였다.
%   주행 중 계측: 109.3 ms -> 24.2 ms (10 Hz 예산의 24%)
%
% 데이터: fig_solvetime.mat  (results/solvetime_summary.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_solvetime`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_solvetime.mat')

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];

figure('Color','w','Position',[80 80 1280 380])

%% (a) 주행 중 계측 - 막대 중앙값, 수염 p95
subplot(1,3,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
med = inloop_stats(:,1);  p95 = inloop_stats(:,3);
cols = [NOM; RES; RES];
for k = 1:3
    bar(k, med(k), 0.5, 'FaceColor', cols(k,:), 'EdgeColor','none');
    plot([k k], [med(k) p95(k)], '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.1);
    plot(k, p95(k), '_', 'Color', [0.25 0.25 0.25], 'MarkerSize', 10, 'LineWidth', 1.1);
    text(k, p95(k)*1.20, sprintf('%.1f', med(k)), ...
         'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
end
plot([0.4 3.6], [budget_ms budget_ms], '--', 'Color', GREY, 'LineWidth', 1.6);
text(3.55, budget_ms*1.35, sprintf('10 Hz budget  %d ms', budget_ms), ...
     'Color', GREY, 'FontSize', 9, 'HorizontalAlignment','right');
text(2, 0.52, 'before', 'HorizontalAlignment','center','FontSize',9,'Color',RES);
text(3, 0.52, 'after',  'HorizontalAlignment','center','FontSize',9,'Color',RES);
set(gca,'XTick',1:3,'XTickLabel',{'Nominal','Residual','Residual'},'YScale','log')
xlim([0.4 3.6]); ylim([0.4 400])
ylabel('Solve time  [ms]   (bar median, whisker p95)')
title('(a)  Measured in closed loop','FontSize',11,'FontWeight','normal')

%% (b) 시간은 SQP 반복 횟수에 정비례한다
subplot(1,3,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
it = sweep(:,1); ms = sweep(:,2);
plot([1 100], [budget_ms budget_ms], '--', 'Color', GREY, 'LineWidth', 1.6);
plot([1 100], [nominal_offline_ms nominal_offline_ms], ':', 'Color', NOM, 'LineWidth', 1.8);
plot(it, ms, '-o', 'Color', RES, 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', RES);
plot(10,  ms(it==10),  'o', 'Color','k', 'MarkerSize', 12, 'LineWidth', 1.6);
plot(100, ms(it==100), 'o', 'Color','k', 'MarkerSize', 12, 'LineWidth', 1.6);
text(10,  ms(it==10)*0.42,  'after',  'FontSize',9,'HorizontalAlignment','center');
text(100, ms(it==100)*2.1,  'before', 'FontSize',9,'HorizontalAlignment','right');
text(1.15, nominal_offline_ms*0.58, 'nominal', 'Color', NOM, 'FontSize', 9);
text(1.15, budget_ms*1.35, sprintf('10 Hz budget  %d ms', budget_ms), ...
     'Color', GREY, 'FontSize', 9);
set(gca,'XScale','log','YScale','log')
xlim([0.85 130]); ylim([0.3 400])
xlabel('SQP iterations per control step'); ylabel('Solve time  [ms]')
title('(b)  Time scales with iteration count','FontSize',11,'FontWeight','normal')

%% (c) 해는 10회에서 이미 수렴 - 나머지 90회는 낭비
subplot(1,3,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
dev = sweep(:,3);
dev(dev <= 0) = 1e-4;                       % log 축 표시용 하한
plot(it, dev, '-s', 'Color', RES, 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', RES);
plot(10, dev(it==10), 'o', 'Color','k', 'MarkerSize', 12, 'LineWidth', 1.6);
text(10, dev(it==10)*8, sprintf('%.4f\\circ', sweep(it==10,3)), ...
     'FontSize',9,'HorizontalAlignment','center');
set(gca,'XScale','log','YScale','log')
xlim([0.85 130]); ylim([5e-5 40])
xlabel('SQP iterations per control step')
ylabel('Steering difference vs 100 iters  [deg]')
title('(c)  Converged already at 10','FontSize',11,'FontWeight','normal')
