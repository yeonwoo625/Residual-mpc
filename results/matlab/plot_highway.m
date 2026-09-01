% 미학습 트랙 일반화 - Hockenheim 학습 -> USA Highway No.1 (재학습 없음)
%
% 이 도로는 차선 3.5 m, 트럭 2.49 m 라 좌우 여유가 0.5 m 뿐이다.
% 그래서 성능 차이가 '오차 감소'가 아니라 '완주 가능 여부'로 나타난다.
%   nominal  : 최대 0.835 m -> 여유 초과, 625 m 에서 도로 이탈
%   residual : 최대 0.213 m -> 931 m 완주
%
% 데이터: fig_highway.mat (요약), fig_highway_traj.mat (궤적)
%         results/highway_summary.py 가 생성
% 실행:   results/matlab/ 폴더 안에서 `plot_highway`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

load('fig_highway.mat')
load('fig_highway_traj.mat')

NOM = [0.12 0.47 0.71];
RES = [0.90 0.49 0.13];
RED = [0.75 0.20 0.18];

figure('Color','w','Position',[80 80 980 380])

%% (a) 최대 횡오차 vs 차선 여유
subplot(1,2,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
bar(1, max_n(1), 0.5, 'FaceColor', NOM, 'EdgeColor','none');
bar(2, max_n(2), 0.5, 'FaceColor', RES, 'EdgeColor','none');
plot([0.4 2.6], [margin margin], '--', 'Color', RED, 'LineWidth',2);
text(2.55, margin+0.05, sprintf('lane margin  %.2f m', margin), ...
     'Color', RED, 'FontSize',10, 'HorizontalAlignment','right');
text(1, max_n(1)+0.055, sprintf('%.3f m  (%.0f%%)', max_n(1), 100*max_n(1)/margin), ...
     'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
text(2, max_n(2)+0.055, sprintf('%.3f m  (%.0f%%)', max_n(2), 100*max_n(2)/margin), ...
     'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
text(1, max_n(1)*0.45, {'left the road','at 625 m'}, ...
     'HorizontalAlignment','center','FontSize',9,'Color','w');
set(gca,'XTick',[1 2],'XTickLabel',{'Nominal MPC','Residual MPC'})
ylabel('Max lateral error  [m]'); ylim([0 max(max_n)*1.35]); xlim([0.4 2.6])

%% (b) 경로거리에 따른 횡오차
subplot(1,2,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
plot([0 path_length], [margin margin], '--', 'Color', RED, 'LineWidth',1.6);
plot(nom_s, abs(nom_n), '-', 'Color', NOM, 'LineWidth',1.5);
plot(res_s, abs(res_n), '-', 'Color', RES, 'LineWidth',1.5);
plot(nom_s(end), abs(nom_n(end)), 'x', 'Color', NOM, 'MarkerSize',12, 'LineWidth',2.5);
text(nom_s(end)-20, abs(nom_n(end))+0.07, 'off road', ...
     'Color', NOM, 'FontSize',9, 'HorizontalAlignment','right');
xlabel('Path distance  s  [m]'); ylabel('|n|  [m]')
xlim([0 path_length]); ylim([0 max(max_n)*1.2])
legend({'lane margin','Nominal MPC','Residual MPC'}, ...
       'Location','northwest','Box','off','FontSize',9)

%% 콘솔 요약
fprintf('\n[미학습 트랙  USA Highway No.1]  학습은 Hockenheim, 재학습 없음\n');
fprintf('  경로 %.0f m, 최소반경 %.1f m, κ 학습분포 밖 %.1f%%\n', ...
        path_length, min_radius, kappa_outside_train_pct);
fprintf('  차선 %.1f m / 트럭 %.2f m -> 여유 %.2f m\n\n', lane_width, truck_width, margin);
for i = 1:2
    fprintf('  %-9s %s  %4.0f m  평균 %.3f  최대 %.3f (여유의 %3.0f%%)\n', ...
        char(labels(i)), char('X'*(1-completed(i))+'O'*completed(i)), ...
        distance_m(i), mean_n(i), max_n(i), 100*max_n(i)/margin);
end
fprintf('\n  평균 %.0f%% 감소, 최대 %.0f%% 감소\n\n', ...
        100*(mean_n(1)-mean_n(2))/mean_n(1), 100*(max_n(1)-max_n(2))/max_n(1));
