% 종방향 잔차를 쓰지 않은 근거 - 세 갈래 증거
%
%   (a) 과도구간의 존재   스로틀 D=1.00 고정인데 가속도가 반복 붕괴 (8회 검출).
%                        차량은 토크컨버터 자동변속기(Auto_Conv, 클러치 Closed)라
%                        클러치 단절이 아니라 기어비 전환에 따른 구동력 교란이다.
%   (b) 예측 대 실제      횡 상관 0.98~0.99 vs 종 0.28. Δv 는 R2=0.074 로
%                        잔차의 7% 만 설명한다.
%   (c) 정속/과도 분해    과도(주행의 8%) 설명불가 59.3%, 정속(88%) 4.4%.
%                        정속의 보정량은 2초 예측구간에서 0.15 m/s 뿐이다.
%
% => 보정이 필요한 구간에서는 학습이 불가능하고, 학습이 가능한 구간에서는
%    보정량이 작다. 따라서 잔차는 횡방향에 집중한다.
%
% ** 주장하지 않는 것 ** "종방향 잔차를 켜면 발산한다"는 검증 기록이 없다.
% 이 그림은 학습 가능성과 보정 실익만 근거로 삼는다.
%
% 데이터: fig_longitudinal_evidence.mat  (results/longitudinal_evidence.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_longitudinal_evidence`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_longitudinal_evidence.mat')

LAT  = [0.12 0.47 0.71];    % 횡방향
LON  = [0.90 0.49 0.13];    % 종방향
GREY = [0.55 0.55 0.55];

DN = 2; DV = 4;             % 채널 인덱스

%% ---- 수치 표 (명령창) ----
fprintf('\n=== (b) Network fit per channel (held-out %d samples) ===\n', n_val);
fprintf('%10s %10s %9s %12s %12s\n', 'channel', 'corr', 'R2', 'RMS red.', 'true RMS');
for j = 1:numel(channel)
    fprintf('%10s %10.3f %9.3f %11.1f%% %12.5f\n', channel{j}, ...
            fit(j,1), fit(j,2), fit(j,3), fit(j,4));
end
fprintf('\n=== (c) Unexplained variance by regime ===\n');
fprintf('%24s %8s %8s %8s\n', 'regime', 'share', 'dv', 'dn');
for i = 1:numel(regime)
    fprintf('%24s %7.0f%% %7.1f%% %7.1f%%\n', regime{i}, ...
            100*regime_frac(i), 100*unexpl(i,1), 100*unexpl(i,2));
end
fprintf('\nSteady-regime dv bias = %+.5f m/s per step -> %+.3f m/s over %d steps\n', ...
        steady_bias, steady_bias*horizon_steps, horizon_steps);
fprintf(['Correction is impossible where it is needed (transient) and negligible ' ...
         'where it is possible (steady).\n\n']);

figure('Color','w','Position',[50 50 1320 400])

%% (a) 스로틀 고정인데 가속도가 무너진다
subplot(1,3,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
h2 = plot(t, D_launch,   '-', 'Color', GREY, 'LineWidth', 1.8);
h1 = plot(t, acc_launch, '-', 'Color', LAT,  'LineWidth', 1.4);
h3 = plot(t(shift_idx), acc_launch(shift_idx), 'v', 'Color', LON, ...
          'MarkerSize', 7, 'MarkerFaceColor', LON);
legend([h2 h1 h3], {'throttle  D', 'acceleration  [m/s^2]', 'ratio change'}, ...
       'Location','northeast','Box','off','FontSize',8.5);
xlim([0 max(t)]); ylim([-0.15 1.75])
xlabel('Time from standstill  [s]'); ylabel('D  /  acceleration  [m/s^2]')
title('(a)  Throttle held, acceleration collapses','FontSize',11,'FontWeight','normal')

%% (b) 예측 대 실제 - 횡과 종을 한 칸에 겹쳐 비교
subplot(1,3,2); hold on; box on
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.9 .9 .9],'GridAlpha',1,'Layer','bottom')
% 축 범위는 각 채널의 실제 RMS 로 정규화해 한 칸에 겹친다
sN = fit(DN,4);  sV = fit(DV,4);
g1 = plot(y_true(:,DN)/sN, y_pred(:,DN)/sN, '.', 'Color', LAT, 'MarkerSize', 4);
g2 = plot(y_true(:,DV)/sV, y_pred(:,DV)/sV, '.', 'Color', LON, 'MarkerSize', 4);
L = 4;
plot([-L L], [-L L], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
text(-L*0.92, L*0.86, sprintf('\\Deltan   r = %.3f', fit(DN,1)), ...
     'FontSize',10,'Color',LAT,'FontWeight','bold');
text(-L*0.92, L*0.68, sprintf('\\Deltav   r = %.3f', fit(DV,1)), ...
     'FontSize',10,'Color',LON,'FontWeight','bold');
text(L*0.95, -L*0.90, 'y = x', 'HorizontalAlignment','right', ...
     'FontSize',9,'Color',[0.3 0.3 0.3]);
axis square; xlim([-L L]); ylim([-L L])
xlabel('Actual residual   (normalised by its RMS)')
ylabel('Network prediction')
title('(b)  Lateral is on the line, longitudinal is not', ...
      'FontSize',11,'FontWeight','normal')

%% (c) 정속 / 과도 분해
subplot(1,3,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
x = (1:numel(regime))'; w = 0.34;
b1 = bar(x-w/2, 100*unexpl(:,1), w, 'FaceColor', LON, 'EdgeColor','none');
b2 = bar(x+w/2, 100*unexpl(:,2), w, 'FaceColor', LAT, 'EdgeColor','none');
for i = 1:numel(regime)
    text(x(i)-w/2, 100*unexpl(i,1)+3, sprintf('%.1f%%', 100*unexpl(i,1)), ...
         'HorizontalAlignment','center','FontSize',9,'Color',LON);
    text(x(i), -6, sprintf('%.0f%% of run', 100*regime_frac(i)), ...
         'HorizontalAlignment','center','FontSize',8.5,'Color',[0.45 0.45 0.45]);
end
legend([b1 b2], {'\Deltav  (longitudinal)','\Deltan  (lateral)'}, ...
       'Location','northwest','Box','off','FontSize',9);
set(gca,'XTick',1:numel(regime),'XTickLabel',{'all','steady','transient'})
xlim([0.4 numel(regime)+0.6]); ylim([-10 70])
ylabel('Unexplained variance  [%]')
title('(c)  Transient is the problem, not the channel', ...
      'FontSize',11,'FontWeight','normal')
text(numel(regime)+0.55, 62, sprintf('steady bias: %+.2f m/s over 2 s', ...
     steady_bias*horizon_steps), 'HorizontalAlignment','right', ...
     'FontSize',8.5,'Color',[0.35 0.35 0.35]);
