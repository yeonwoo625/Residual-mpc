% 저마찰 노면 (mu = 0.3) - 잔차 보정의 작동 조건
%
% 핵심: 잔차는 타이어에 여유가 있을 때만 작동한다.
%   마찰 사용률 73% (v=9)  -> 급코너 오차 34% 감소, 완주
%   마찰 사용률 90% (v=10) -> 조향 포화, 4회 시도 전부 이탈
%
% 데이터: fig_friction.mat  (results/friction_summary.py 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_friction`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.
%
% 색: 파랑(nominal) / 주황(residual), 이탈은 옅게 + X 표시.

load('fig_friction.mat')

NOM = [0.12 0.47 0.71];
RES = [0.90 0.49 0.13];
GRY = [0.45 0.45 0.45];

figure('Color','w','Position',[80 80 960 380])

%% (a) 완주한 조건에서의 성능 비교
subplot(1,2,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
v9n  = corner_rms(v_target==9  & is_residual==0 & completed==1);
v9r  = corner_rms(v_target==9  & is_residual==1 & completed==1);
v10n = corner_rms(v_target==10 & is_residual==0 & completed==1);
bar(1, v10n, 0.5, 'FaceColor', NOM, 'EdgeColor','none');
bar(2.6, v9n, 0.5, 'FaceColor', NOM, 'EdgeColor','none');
bar(3.2, v9r, 0.5, 'FaceColor', RES, 'EdgeColor','none');
text(1,   v10n+0.06, sprintf('%.3f', v10n), 'HorizontalAlignment','center','FontSize',10);
text(2.6, v9n +0.06, sprintf('%.3f', v9n),  'HorizontalAlignment','center','FontSize',10);
text(3.2, v9r +0.06, sprintf('%.3f', v9r),  'HorizontalAlignment','center','FontSize',10);
text(2.9, v9n+0.28, sprintf('-%.0f%%', 100*(v9n-v9r)/v9n), ...
     'HorizontalAlignment','center','FontWeight','bold','FontSize',11,'Color',GRY);
text(1, v10n*0.45, {'residual','all diverged'}, 'HorizontalAlignment','center', ...
     'FontSize',9,'Color',RES);
set(gca,'XTick',[1 2.9],'XTickLabel',{'v = 10  (util 90%)','v = 9  (util 73%)'})
ylabel('Corner RMS lateral error  [m]'); ylim([0 max(corner_rms)*1.25]); xlim([0.4 3.8])
h(1)=patch(nan,nan,NOM,'EdgeColor','none'); h(2)=patch(nan,nan,RES,'EdgeColor','none');
legend(h,{'Nominal MPC','Residual MPC'},'Location','northeast','Box','off','FontSize',9)

%% (b) 왜 v=10 에서 실패했나 - 조향 실효이득과 포화
subplot(1,2,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
for i = 1:numel(steer_gain)
    c = NOM; if is_residual(i), c = RES; end
    mk = 'o'; if ~completed(i), mk = 'x'; end
    plot(friction_util(i), steer_gain(i), mk, 'Color', c, ...
         'MarkerFaceColor', c, 'MarkerSize',10, 'LineWidth',1.8);
end
plot([80 80],[0 0.8],':','Color',GRY,'LineWidth',1.2);
text(81, 0.72, {'tire saturation','(util > 80%)'}, 'FontSize',9,'Color',GRY);
xlabel('Friction utilization at sharpest corner  [%]')
ylabel('Effective steering gain  (actual / model)')
xlim([65 95]); ylim([0 0.8])
h2(1)=plot(nan,nan,'o','Color',GRY,'MarkerFaceColor',GRY,'MarkerSize',8);
h2(2)=plot(nan,nan,'x','Color',GRY,'MarkerSize',10,'LineWidth',1.8);
legend(h2,{'completed','diverged'},'Location','southwest','Box','off','FontSize',9)

%% 콘솔 요약
fprintf('\n[mu = %.1f 저마찰]\n', mu);
for i = 1:numel(corner_rms)
    fprintf('  %-26s %s  corner %.3f  util %3.0f%%  gain %.2f  sat %4.1f%%\n', ...
        char(labels(i)), char('X'*(1-completed(i)) + 'O'*completed(i)), ...
        corner_rms(i), friction_util(i), steer_gain(i), steer_sat_pct(i));
end
fprintf('\n  v=9  (util 73%%) : %.3f -> %.3f  (-%.1f%%)\n', v9n, v9r, 100*(v9n-v9r)/v9n);
fprintf('  v=10 (util 90%%) : nominal %.3f 완주, residual 4회 모두 이탈\n\n', v10n);
