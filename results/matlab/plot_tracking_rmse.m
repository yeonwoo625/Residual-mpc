% Tracking RMSE 표 - X / Y / Yaw, 조건별 nominal vs residual
%
% 논문 표 형식으로 그린다. 각 조건에서 더 좋은(작은) 값을 굵게 표시한다.
%
% 지표 정의
%   Y  (m)   횡오차   = Frenet n     (기준 경로에서 옆으로 벗어난 거리) RMSE
%   Yaw(rad) 헤딩오차 = Frenet alpha (경로 접선 대비 차체 방향)        RMSE
%   X  (m)   종방향 진행오차 = (실제 진행거리) - (목표속도 x 경과시간) RMSE
%
% ** X 주의 ** 이 MPC 는 trajectory tracking 이 아니라 path following 이다.
% 종방향 목표가 s_target = s + v*dt*(i+1) 로 현재 위치 기준 상대값이라 절대
% 시간 기준이 없고, 종방향 지연을 되돌릴 수단이 없어 X 는 설계상 누적된다.
% Y 보다 두 자리 큰 것은 제어 실패가 아니라 이 구조 탓이다. 종방향 성능은
% SHOW_EXTRA=1 로 켜지는 속도추종 RMSE 와 랩타임으로 읽는 편이 정확하다.
%
% 데이터: fig_tracking_rmse.mat  (results/tracking_rmse.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_tracking_rmse`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_tracking_rmse.mat')

SHOW_EXTRA = 1;    % 1 = 속도추종 RMSE / 랩타임 / 실제 제어율 열을 함께 표시
                   %     제어율은 잔차 주행 일부가 6.5 Hz 라 공정성 확인에 필요하다
TITLE = 'Tracking RMSE Comparison';

nS = numel(scenario);
if SHOW_EXTRA
    hdr  = {'Scenario','Method','X Error (m)','Y Error (m)','Yaw Error (rad)', ...
            'Speed RMSE (m/s)','Lap (s)','Rate (Hz)'};
    dat  = {x_rmse, y_rmse, yaw_rmse, v_rmse, lap_time, rate_hz};
    fmt  = {'%.2f','%.3f','%.4f','%.3f','%.1f','%.1f'};
    lower_better = [1 1 1 1 1 0];          % 0 = 굵게 표시하지 않음
    colx = [0.030 0.235 0.400 0.505 0.625 0.760 0.860 0.945];
else
    hdr  = {'Scenario','Method','X Error (m)','Y Error (m)','Yaw Error (rad)'};
    dat  = {x_rmse, y_rmse, yaw_rmse};
    fmt  = {'%.2f','%.3f','%.4f'};
    lower_better = [1 1 1];
    colx = [0.05 0.32 0.55 0.72 0.90];
end
nC = numel(dat);

rowH  = 1/(2*nS + 4.2);                 % 헤더 + 데이터 + 여백
figW  = 240 + 105*nC;
figure('Color','w','Position',[80 80 figW 90+34*(2*nS+2)])
axes('Position',[0 0 1 1]); hold on; axis off
xlim([0 1]); ylim([0 1]); set(gca,'YDir','reverse')

yt = @(k) (k+1.5)*rowH;                 % k 번째 줄의 y 좌표
rule = @(y,w) line([0.02 0.98],[y y],'Color','k','LineWidth',w);

% 제목
text(0.5, 0.6*rowH, TITLE, 'HorizontalAlignment','center', ...
     'FontSize',12,'FontWeight','bold');

% 헤더
rule(yt(-0.6), 1.4);
for c = 1:numel(hdr)
    al = 'center'; if c <= 2, al = 'left'; end
    text(colx(c), yt(0), hdr{c}, 'HorizontalAlignment',al, ...
         'FontSize',10,'FontWeight','bold');
end
rule(yt(0.6), 0.8);

% 본문 - 조건마다 두 줄(nominal / residual)
for i = 1:nS
    for j = 1:2
        r = 2*(i-1) + j;
        y = yt(r);
        if j == 1
            text(colx(1), y, scenario{i}, 'HorizontalAlignment','left','FontSize',10);
        end
        text(colx(2), y, method{j}, 'HorizontalAlignment','left','FontSize',10);
        for c = 1:nC
            v = dat{c}(i,j);
            if lower_better(c) && dat{c}(i,j) < dat{c}(i,3-j)
                fw = 'bold';
            else
                fw = 'normal';
            end
            text(colx(2+c), y, sprintf(fmt{c}, v), ...
                 'HorizontalAlignment','center','FontSize',10,'FontWeight',fw);
        end
    end
    if i < nS
        line([0.02 0.98],[yt(2*i+0.5) yt(2*i+0.5)], ...
             'Color',[0.82 0.82 0.82],'LineWidth',0.5);
    end
end
rule(yt(2*nS+0.6), 1.4);

% 각주 - 정의와 한계를 표 안에 남긴다
note = ['X = longitudinal progress error (path following: no absolute time ' ...
        'reference, so it accumulates - not comparable to trajectory-tracking X).'];
text(0.02, yt(2*nS+1.5), note, 'HorizontalAlignment','left', ...
     'FontSize',8,'Color',[0.4 0.4 0.4],'Interpreter','none');
if SHOW_EXTRA && any(rate_hz(:) < 9)
    text(0.02, yt(2*nS+2.2), ...
        'Residual runs below 10 Hz predate the solver fix (109 ms solve). Only v=14 matches rates.', ...
        'HorizontalAlignment','left','FontSize',8,'Color',[0.4 0.4 0.4],'Interpreter','none');
end
