% 적재 질량이 왜 MLP 입력으로 불필요한가 - 관측 가능성 + 조건부 잉여성
%
% 왼쪽부터: 질량이 보인다 -> n 덕분이다 -> 그래서 잉여다
%
% 데이터: fig_payload_observability.mat  (results/payload_observability.py 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_payload_observability`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.
%
% 색: 파랑(입력이 설명) / 주황(질량이 추가로 설명). 적록색약 안전.

load('fig_payload_observability.mat')

BLU = [0.12 0.47 0.71];
ORA = [0.90 0.49 0.13];
GRY = [0.45 0.45 0.45];

figure('Color','w','Position',[80 80 1080 350])

%% (a) 입력만으로 예측한 질량 vs 실제 질량
subplot(1,3,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
lim = [28 60];
plot(lim, lim, ':', 'Color', GRY, 'LineWidth',1.2);          % 1:1 선
for i = 1:numel(mass_classes)
    plot([mass_classes(i) mass_classes(i)], [pred_lo(i) pred_hi(i)], ...
         '-', 'Color', BLU, 'LineWidth',2);
end
plot(mass_classes, pred_mean, 'o', 'Color', BLU, ...
     'MarkerFaceColor', BLU, 'MarkerSize',8);
xlim(lim); ylim(lim); axis square
xlabel('True payload mass  [t]'); ylabel('Predicted from state  [t]')
text(30, 57, sprintf('R^2 = %.2f', r2_knn), 'FontSize',11,'FontWeight','bold');
text(30, 54, sprintf('mean error %.1f t', mass_err_t), 'FontSize',10,'Color',GRY);
legend({'1:1','5-95%','mean'},'Location','southeast','Box','off','FontSize',9)

%% (b) 어느 입력이 질량 정보를 나르는가
subplot(1,3,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
[srt, ord] = sort(drop_r2, 'descend');
for i = 1:numel(srt)
    bar(i, srt(i), 0.6, 'FaceColor', BLU, 'EdgeColor','none');
end
set(gca,'XTick',1:numel(srt),'XTickLabel',feat_names(ord))
ylabel('Drop in R^2 when removed')
xlim([0.4 numel(srt)+0.6]); ylim([0 max(srt)*1.25])
text(1, srt(1)*1.10, 'lateral error carries the mass', ...
     'FontSize',9,'Color',GRY);

%% (c) 입력을 고정하면 질량이 남기는 정보
subplot(1,3,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
bar(1, 100, 0.55, 'FaceColor', BLU, 'EdgeColor','none');
bar(2, cond_r2_pct(1), 0.55, 'FaceColor', ORA, 'EdgeColor','none');
bar(3, cond_r2_pct(2), 0.55, 'FaceColor', ORA, 'EdgeColor','none');
text(1, 103, 'state', 'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
text(2, 8, sprintf('%.2f%%', cond_r2_pct(1)), 'HorizontalAlignment','center','FontSize',10);
text(3, 8, sprintf('%.2f%%', cond_r2_pct(2)), 'HorizontalAlignment','center','FontSize',10);
set(gca,'XTick',1:3,'XTickLabel',{'input','+mass (\Deltan)','+mass (\Delta\alpha)'})
ylabel('Share of residual explained  [%]'); ylim([0 118]); xlim([0.4 3.6])

%% 콘솔 요약
fprintf('\n[관측 가능성]  입력 6차원 -> 질량\n');
fprintf('  선형 R2 %.3f / 최근접이웃 R2 %.3f, 평균오차 %.1f t, ±4t 적중 %.0f%%\n', ...
        r2_linear, r2_knn, mass_err_t, hit_4t);
fprintf('  기여 1위: %s (R2 %.3f 하락)\n', char(feat_names(ord(1))), srt(1));
fprintf('\n[조건부 잉여성]  입력 고정(k=%d) 시 질량의 추가 설명력\n', k_neighbors);
fprintf('  dn %.2f%% / da %.2f%%  -> 질량은 새로운 정보가 아니다\n\n', ...
        cond_r2_pct(1), cond_r2_pct(2));
