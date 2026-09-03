% 개루프 예측 오차 - 잔차가 '모델'을 개선하는가, 어디까지 개선하는가
%
% 지금까지의 증거는 전부 폐루프 추종 오차였다. 이 그림은 제어기를 거치지 않고
% 모델만으로 N 스텝을 적분해 실제 상태와 비교한다. 주장("잔차가 kinematic 모델
% 오차를 보정한다")과 증거가 직접 맞물리는 유일한 지표다.
%
% (a) 학습 분포 안(Hockenheim v=12): 예측구간이 길수록 격차가 벌어진다.
%     MPC 예측구간 2.0 s 에서 횡오차 예측이 1.62 m -> 0.18 m (89% 감소).
% (b) 조건별 개선율: 학습 분포에서 멀어질수록 단조롭게 무너지고, 미학습 트랙과
%     저마찰에서는 오히려 나빠진다. 그 두 조건의 폐루프 이득은 '모델이
%     정확해져서'가 아니라는 뜻이다(가중치만 올린 nominal 이 잔차를 따라잡는
%     results/b1/ 의 결과와 같은 방향).
%
% 데이터: fig_openloop.mat  (results/openloop_prediction.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_openloop`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_openloop.mat')

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];
GRN  = [0.00 0.62 0.45];      % Okabe-Ito bluish green (색약 안전)
PUR  = [0.80 0.47 0.65];      % Okabe-Ito reddish purple

h  = horizon_s(:);
DN = 2;                        % 채널 인덱스: 1=ds 2=dn 3=dalpha 4=dv

%% ---- 수치 표 출력 (명령창) ----
fprintf('\n=== Open-loop prediction error, lateral dn [m] ===\n');
fprintf('%-20s %-16s', 'case', 'distribution');
fprintf('%9.1fs', h); fprintf('\n');
for c = 1:numel(case_)
    fprintf('%-20s %-16s', case_{c}, dist{c});
    fprintf('%10.4f', squeeze(err(c,:,1,DN)));       % nominal
    fprintf('   <- nominal\n');
    fprintf('%-20s %-16s', '', '');
    fprintf('%10.4f', squeeze(err(c,:,2,DN)));       % +residual
    fprintf('   <- +residual\n');
    fprintf('%-20s %-16s', '', 'reduction');
    fprintf('%9.0f%%', improve_dn(c,:)); fprintf('\n');
end
fprintf(['\nPositive reduction = residual improves the model. ' ...
         'It degrades away from the training data.\n\n']);

figure('Color','w','Position',[80 80 1000 380])

%% (a) 학습 분포 안에서의 예측 오차 (Hockenheim v=12)
subplot(1,2,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
e_nom = squeeze(err(1,:,1,DN)); e_nom = e_nom(:);
e_res = squeeze(err(1,:,2,DN)); e_res = e_res(:);
p1 = plot(h, e_nom, '-o', 'Color', NOM, 'LineWidth', 2, 'MarkerSize', 6, ...
          'MarkerFaceColor', NOM);
p2 = plot(h, e_res, '-s', 'Color', RES, 'LineWidth', 2, 'MarkerSize', 6, ...
          'MarkerFaceColor', RES);
for k = 1:numel(h)
    text(h(k), e_res(k)*0.62, sprintf('-%.0f%%', improve_dn(1,k)), ...
         'Color', RES, 'FontSize', 9, 'HorizontalAlignment','center');
end
legend([p1 p2], {'Nominal model','Nominal + residual'}, ...
       'Location','northwest','Box','off','FontSize',9);
set(gca,'YScale','log')
xlim([0 2.15]); ylim([5e-3 3])
xlabel('Prediction horizon  [s]')
ylabel('Open-loop lateral prediction error  [m]')
title('(a)  In-distribution (Hockenheim v=12)','FontSize',11,'FontWeight','normal')

%% (b) 학습 분포에서 멀어질수록
subplot(1,2,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
cols = [RES; GRN; NOM; PUR];
mk   = {'-s','-o','-^','-v'};
hh = zeros(1, size(improve_dn,1));
for c = 1:size(improve_dn,1)
    yv = improve_dn(c,:); yv = yv(:);
    hh(c) = plot(h, yv, mk{c}, 'Color', cols(c,:), ...
                 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', cols(c,:));
end
plot([0 2.15], [0 0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0);
text(2.10, 6, 'residual helps', 'HorizontalAlignment','right', ...
     'FontSize',9,'Color',GREY);
text(2.10, -12, 'residual hurts', 'HorizontalAlignment','right', ...
     'FontSize',9,'Color',GREY);
lbl = cell(1, numel(case_));
for c = 1:numel(case_)
    lbl{c} = sprintf('%s  [%s]', case_{c}, dist{c});
end
legend(hh, lbl, 'Location','southwest','Box','off','FontSize',8.5);
xlim([0 2.15]); ylim([-100 100])
xlabel('Prediction horizon  [s]')
ylabel('Lateral prediction error reduction  [%]')
title('(b)  Validity shrinks away from training data','FontSize',11,'FontWeight','normal')
