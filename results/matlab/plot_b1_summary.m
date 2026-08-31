% B1 실험 종합 - nominal 가중치 튜닝 vs residual MPC
%
% 데이터: fig_b1_summary.mat  (results/b1_summary.py 가 생성)
%   speed       속도 스윕 (Q_n=1e-4): v_target v_actual nom res improve chat_nom chat_res
%   qn_sweep    가중치 스윕 (v=10, nominal): qn corner mean chatter v laptime
%   payload     적재 검증 (Q_n=1e-2): mass corner mean chatter
%   interaction 높은 gain 에서 잔차 세기: scale corner mean chatter labels
%
% 색: 파랑(nominal) / 주황(residual). 적록색약 안전.
%
% 실행: results/matlab/ 폴더 안에서 `plot_b1_summary`
% 요구: base MATLAB 만 (툴박스 불필요). R2013a 이상.

load('fig_b1_summary.mat')

NOM = [0.12 0.47 0.71];      % 파랑
RES = [0.90 0.49 0.13];      % 주황
GRY = [0.45 0.45 0.45];

%% ===== Fig 1 : 속도별 nominal vs residual =====
figure('Color','w','Position',[80 80 900 380])

subplot(1,2,1); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
x = 1:numel(speed.v_target);
bar(x-0.18, speed.nom, 0.34, 'FaceColor', NOM, 'EdgeColor','none');
bar(x+0.18, speed.res, 0.34, 'FaceColor', RES, 'EdgeColor','none');
for i = x
    text(i, max(speed.nom(i),speed.res(i))+0.06, ...
         sprintf('-%.0f%%', speed.improve(i)), 'HorizontalAlignment','center', ...
         'FontWeight','bold','FontSize',10,'Color',GRY);
end
set(gca,'XTick',x,'XTickLabel',arrayfun(@(z) sprintf('%.1f',z), speed.v_target, 'UniformOutput',false))
xlabel('Target speed  [m/s]'); ylabel('Corner RMS lateral error  [m]')
ylim([0 max(speed.nom)*1.25]); xlim([0.4 numel(x)+0.6])
legend({'Nominal MPC','Residual MPC'},'Location','northwest','Box','off','FontSize',10)

subplot(1,2,2); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
bar(x-0.18, speed.chat_nom, 0.34, 'FaceColor', NOM, 'EdgeColor','none');
bar(x+0.18, speed.chat_res, 0.34, 'FaceColor', RES, 'EdgeColor','none');
set(gca,'XTick',x,'XTickLabel',arrayfun(@(z) sprintf('%.1f',z), speed.v_target, 'UniformOutput',false))
xlabel('Target speed  [m/s]'); ylabel('Steering chatter  RMS(d\delta/dt)')
xlim([0.4 numel(x)+0.6])
legend({'Nominal MPC','Residual MPC'},'Location','northwest','Box','off','FontSize',10)

%% ===== Fig 2 : 가중치 스윕 + 적재 검증 =====
figure('Color','w','Position',[120 120 900 380])

subplot(1,2,1); hold on; box off
set(gca,'FontSize',11,'TickDir','out','XScale','log','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
plot(qn_sweep.qn, qn_sweep.corner, '-o', 'Color', NOM, 'LineWidth',2, ...
     'MarkerFaceColor', NOM, 'MarkerSize',7);
plot([7e-5 1.4e-2], [speed.res(1) speed.res(1)], '--', 'Color', RES, 'LineWidth',2);
text(1.1e-4, speed.res(1)+0.035, 'Residual MPC (Q_n = 10^{-4})', 'Color', RES, 'FontSize',10);
xlabel('Lateral error weight  Q_n'); ylabel('Corner RMS lateral error  [m]')
xlim([7e-5 1.4e-2]); ylim([0 max(qn_sweep.corner)*1.15])
text(1.1e-4, 0.90*max(qn_sweep.corner), sprintf('-%.0f%% by tuning alone', ...
     100*(1-min(qn_sweep.corner)/max(qn_sweep.corner))), 'FontSize',10,'Color',GRY);

subplot(1,2,2); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
bar(1:3, payload.corner, 0.5, 'FaceColor', NOM, 'EdgeColor','none');
for i = 1:3
    text(i, payload.corner(i)+0.008, sprintf('%.3f', payload.corner(i)), ...
         'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
end
set(gca,'XTick',1:3,'XTickLabel',arrayfun(@(z) sprintf('%d t',z), payload.mass, 'UniformOutput',false))
xlabel('Payload condition'); ylabel('Corner RMS lateral error  [m]')
ylim([0 max(payload.corner)*1.35]); xlim([0.4 3.6])
legend({'Nominal, single tuned gain (Q_n = 10^{-2})'}, ...
       'Location','northwest','Box','off','FontSize',10)

%% ===== 콘솔 요약 =====
fprintf('\n[속도 스윕]  Q_n=1e-4 고정\n');
for i = 1:numel(speed.v_target)
    fprintf('  v=%.1f : nominal %.3f -> residual %.3f  (-%.1f%%)\n', ...
            speed.v_target(i), speed.nom(i), speed.res(i), speed.improve(i));
end
fprintf('\n[가중치 스윕]  v=10, nominal 단독\n');
fprintf('  Q_n %.0e -> %.0e : %.3f -> %.3f  (-%.0f%%)\n', ...
        qn_sweep.qn(1), qn_sweep.qn(end), qn_sweep.corner(1), qn_sweep.corner(end), ...
        100*(1-qn_sweep.corner(end)/qn_sweep.corner(1)));
fprintf('\n[적재 검증]  단일 gain Q_n=1e-2\n');
fprintf('  32/48/56 t : %.3f / %.3f / %.3f  (편차 %.0f%%)\n\n', ...
        payload.corner(1), payload.corner(2), payload.corner(3), ...
        100*(max(payload.corner)-min(payload.corner))/min(payload.corner));
