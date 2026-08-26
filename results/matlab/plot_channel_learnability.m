% 채널별 잔차 학습 가능성 - 달성치 vs noise ceiling
%
% 메시지: 횡방향은 이미 한계까지 짜냈고, 종방향은 한계 자체가 낮다.
%         → 종방향을 잔차 학습에서 뺀 것은 변명이 아니라 측정된 설계 근거.
%
% 막대  = 신경망이 실제로 줄인 비율 (슬라이드 18의 그 값)
% 가로선 = noise ceiling, 어떤 모델도 넘을 수 없는 상한
%
% 색: 파랑/주황 (적록색약 안전, IEEE 권고). 초록/빨강은 쓰지 말 것.

load('fig_channel_learnability.mat')

LONG = [0.90 0.49 0.13];      % 주황 - 종방향
LAT  = [0.12 0.47 0.71];      % 파랑 - 횡방향
CEIL = [0.15 0.15 0.15];      % 진회색 - ceiling
lbl  = {'\Deltas', '\Deltav', '\Deltan', '\Delta\alpha'};
col  = [LONG; LONG; LAT; LAT];

figure('Color','w','Position',[100 100 620 430]); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on','GridColor',[.85 .85 .85], ...
        'GridAlpha',1,'Layer','bottom')

% --- 막대 (달성치) ---
for i = 1:4
    bar(i, achieved(i), 0.55, 'FaceColor', col(i,:), 'EdgeColor','none');
    text(i, achieved(i)-4, sprintf('%.0f%%', achieved(i)), 'Color','w', ...
         'FontWeight','bold', 'FontSize',11, 'HorizontalAlignment','center');
end

% --- noise ceiling (가로선 + 값) ---
for i = 1:4
    plot([i-0.34 i+0.34], [ceiling(i) ceiling(i)], '-', ...
         'Color', CEIL, 'LineWidth', 2.2);
    text(i, ceiling(i)+3.5, sprintf('%.0f%%', ceiling(i)), 'Color', CEIL, ...
         'FontSize',10, 'HorizontalAlignment','center');
end

% --- 종/횡 구분 ---
plot([2.5 2.5], [0 100], ':', 'Color',[.6 .6 .6], 'LineWidth',1);
text(1.5, -13, 'Longitudinal', 'HorizontalAlignment','center', ...
     'FontSize',11, 'FontWeight','bold', 'Color', LONG);
text(3.5, -13, 'Lateral',      'HorizontalAlignment','center', ...
     'FontSize',11, 'FontWeight','bold', 'Color', LAT);

set(gca,'XTick',1:4,'XTickLabel',lbl,'XLim',[0.45 4.55],'YLim',[0 100])
ylabel('Residual reduction by NN  [%]')

% --- 범례 (색만으로 구분하지 않도록 명시) ---
h(1) = patch(nan,nan,LAT,'EdgeColor','none');
h(2) = plot(nan,nan,'-','Color',CEIL,'LineWidth',2.2);
legend(h, {'Achieved (trained MLP)','Noise ceiling (upper bound)'}, ...
       'Location','northwest','Box','off','FontSize',10);

% --- 콘솔 요약 ---
fprintf('\n%5s %10s %10s %10s\n','ch','achieved','ceiling','달성/한계');
for i = 1:4
    fprintf('%5s %9.1f%% %9.1f%% %9.0f%%\n', ...
            char(channel(i)), achieved(i), ceiling(i), 100*achieved(i)/ceiling(i));
end
fprintf('\n설명 불가 비율(분산): 횡 %.1f%%  vs  종 %.1f%%  →  %.0f배\n\n', ...
        unexplained(3), unexplained(2), unexplained(2)/unexplained(3));
