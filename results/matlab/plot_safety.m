% 안전성 / 실패 모드 - 포화 한계(clamp)의 작동 빈도
%
% 잔차 출력에는 물리적 상한이 이미 걸려 있다 (mpc/residual_model_wrapper.py):
%   clamp = [ds 0.2, dn 0.1, da 0.05, dv 0.4]  (0.1 s 스텝당)
% 제어에 실제 주입되는 것은 dn, da 뿐이다 (B 행렬이 ds/dv 행을 0 으로 매핑).
%
% 핵심: 포화 빈도가 정상 주행에서는 5~7% 인데 불안정 주행에서는 3배로 올라가고,
%       조향 채터와 상관 +0.995 다. 런타임 이상 징후 지표로 쓸 수 있다.
%
% 데이터: fig_safety.mat  (results/safety_failure_modes.py 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_safety`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

load('fig_safety.mat')

OK  = [0.12 0.47 0.71];      % 파랑 - 정상 주행
BAD = [0.90 0.49 0.13];      % 주황 - 불안정 주행
GRY = [0.45 0.45 0.45];

unstable = chatter > 0.15;   % 채터 임계 (정상 주행은 0.06 내외)

figure('Color','w','Position',[80 80 940 380])

%% (a) 주행별 포화 빈도
subplot(1,2,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
for i = 1:numel(sat_dn_pct)
    c = OK; if unstable(i), c = BAD; end
    bar(i, sat_dn_pct(i), 0.6, 'FaceColor', c, 'EdgeColor','none');
    text(i, sat_dn_pct(i)+0.7, sprintf('%.1f%%', sat_dn_pct(i)), ...
         'HorizontalAlignment','center','FontSize',9);
end
set(gca,'XTick',1:numel(sat_dn_pct),'XTickLabel',1:numel(sat_dn_pct))
xlabel('Run #'); ylabel('\Deltan saturation rate  [%]')
ylim([0 max(sat_dn_pct)*1.3]); xlim([0.4 numel(sat_dn_pct)+0.6])
h(1) = patch(nan,nan,OK,'EdgeColor','none');
h(2) = patch(nan,nan,BAD,'EdgeColor','none');
legend(h, {'stable','unstable (chatter > 0.15)'}, ...
       'Location','northwest','Box','off','FontSize',9)

%% (b) 포화 빈도 vs 조향 채터
subplot(1,2,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on','XGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
for i = 1:numel(sat_dn_pct)
    c = OK; if unstable(i), c = BAD; end
    plot(sat_dn_pct(i), chatter(i), 'o', 'Color', c, ...
         'MarkerFaceColor', c, 'MarkerSize',9);
end
p = polyfit(sat_dn_pct, chatter, 1);
xs = linspace(min(sat_dn_pct)-1, max(sat_dn_pct)+1, 20);
plot(xs, polyval(p, xs), '--', 'Color', GRY, 'LineWidth',1.4);
R = corrcoef(sat_dn_pct, chatter);
text(6, max(chatter)*0.92, sprintf('r = %+.3f', R(1,2)), ...
     'FontSize',11,'FontWeight','bold');
xlabel('\Deltan saturation rate  [%]'); ylabel('Steering chatter  RMS(d\delta/dt)')
xlim([min(sat_dn_pct)-1.5 max(sat_dn_pct)+1.5])

%% 콘솔 요약
fprintf('\n[포화 한계]  clamp = [%.2f %.2f %.2f %.2f]  (0.1 s 스텝당)\n', clamp);
for i = 1:numel(sat_dn_pct)
    fprintf('  %d) %-26s  dn 포화 %5.1f%%   채터 %.4f\n', ...
            i, char(run_labels(i)), sat_dn_pct(i), chatter(i));
end
R = corrcoef(sat_dn_pct, chatter);
fprintf('\n  포화 빈도 <-> 조향 채터 상관 r = %+.3f\n', R(1,2));
fprintf('  -> 포화 빈도를 런타임 이상 징후 지표로 사용 가능\n\n');
