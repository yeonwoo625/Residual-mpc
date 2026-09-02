% Tracking RMSE 표 - 일반화 축별 X / Y / Yaw, nominal vs residual
%
% 논문의 일반화 축을 한 표에 담는다.
%   Speed       10 / 11.5 / 12 / 14 m/s   (14 는 학습 상한 12.23 밖 = 외삽)
%   Payload     32 / 48 / 56 t            (48 t 는 Speed v=10 과 같은 주행)
%   Track       USA Highway No.1          (재학습 없음)
%   Friction    mu = 0.3                  (완주한 v=9 만)
%   Steer rate  15 deg/s                  (나머지 행은 기본 57.3 deg/s)
% 각 조건에서 더 좋은(작은) 값을 굵게 표시한다.
%
% 지표 정의
%   Y  (m)   횡오차   = Frenet n     (기준 경로에서 옆으로 벗어난 거리) RMSE
%   Yaw(rad) 헤딩오차 = Frenet alpha (경로 접선 대비 차체 방향)        RMSE
%   X  (m)   종방향 진행오차 = (실제 진행거리) - (목표속도 x 경과시간) RMSE
%
% ** X 주의 ** 이 MPC 는 trajectory tracking 이 아니라 path following 이다.
% 종방향 목표가 s_target = s + v*dt*(i+1) 로 현재 위치 기준 상대값이라 절대
% 시간 기준이 없고, 종방향 지연을 되돌릴 수단이 없어 X 는 설계상 누적된다.
% Y 보다 큰 것은 제어 실패가 아니라 이 구조 탓이며, 시간 궤적을 쫓는 논문의
% X 와 나란히 놓으면 안 된다. 종방향 성능은 SHOW_EXTRA 로 켜지는 속도추종
% RMSE 와 랩타임으로 읽는 편이 정확하다.
%
% 데이터: fig_tracking_rmse.mat  (results/tracking_rmse.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_tracking_rmse`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_tracking_rmse.mat')

SHOW_EXTRA = 1;    % 1 = 속도추종 RMSE / 랩타임 / 실제 제어율 열을 함께 표시.
                   %     제어율은 잔차 주행 다수가 6.5 Hz 라(solve 109 ms, 수정 전)
                   %     공정성 확인에 필요하다. v=14 만 양쪽 10 Hz.
FS         = 9;    % 본문 글씨 크기
TITLE = 'Tracking RMSE Comparison';

nS = numel(scenario);
if SHOW_EXTRA
    hdr = {'Axis','Scenario','Method','X Error (m)','Y Error (m)','Yaw Error (rad)', ...
           'Speed RMSE (m/s)','Lap (s)','Rate (Hz)'};
    dat = {x_rmse, y_rmse, yaw_rmse, v_rmse, lap_time, rate_hz};
    fmt = {'%.2f','%.3f','%.4f','%.3f','%.1f','%.1f'};
    lower_better = [1 1 1 1 1 0];            % 0 = 굵게 표시하지 않음
    colx = [0.022 0.115 0.290 0.425 0.520 0.632 0.760 0.855 0.940];
else
    hdr = {'Axis','Scenario','Method','X Error (m)','Y Error (m)','Yaw Error (rad)'};
    dat = {x_rmse, y_rmse, yaw_rmse};
    fmt = {'%.2f','%.3f','%.4f'};
    lower_better = [1 1 1];
    colx = [0.03 0.15 0.38 0.60 0.75 0.92];
end
nC = numel(dat);

nRow = 2*nS + 4.6;
rowH = 1/nRow;
figW = 260 + 108*nC;
figure('Color','w','Position',[60 40 figW 62+26*(2*nS+3)])
% axis off 를 쓰려면 axis 라는 변수가 없어야 한다 -> .mat 필드는 gen_axis
axes('Position',[0 0 1 1]); hold on; axis off
xlim([0 1]); ylim([0 1]); set(gca,'YDir','reverse')

yt   = @(k) (k+1.7)*rowH;
rule = @(y,w) line([0.015 0.985],[y y],'Color','k','LineWidth',w);

text(0.5, 0.75*rowH, TITLE, 'HorizontalAlignment','center', ...
     'FontSize',12,'FontWeight','bold');

rule(yt(-0.6), 1.4);
for c = 1:numel(hdr)
    al = 'center'; if c <= 3, al = 'left'; end
    text(colx(c), yt(0), hdr{c}, 'HorizontalAlignment',al, ...
         'FontSize',FS,'FontWeight','bold');
end
rule(yt(0.6), 0.8);

for i = 1:nS
    newAxis = (i == 1) || ~strcmp(gen_axis{i}, gen_axis{i-1});
    for j = 1:2
        y = yt(2*(i-1) + j);
        if j == 1
            if newAxis
                text(colx(1), y, gen_axis{i}, 'HorizontalAlignment','left', ...
                     'FontSize',FS,'FontAngle','italic','Color',[0.25 0.25 0.25]);
            end
            text(colx(2), y, scenario{i}, 'HorizontalAlignment','left','FontSize',FS);
        end
        text(colx(3), y, method{j}, 'HorizontalAlignment','left','FontSize',FS);
        for c = 1:nC
            if lower_better(c) && dat{c}(i,j) < dat{c}(i,3-j)
                fw = 'bold';
            else
                fw = 'normal';
            end
            text(colx(3+c), y, sprintf(fmt{c}, dat{c}(i,j)), ...
                 'HorizontalAlignment','center','FontSize',FS,'FontWeight',fw);
        end
    end
    % 축이 바뀌는 곳은 진한 선, 같은 축 안은 옅은 선
    if i < nS
        yl = yt(2*i + 0.5);
        if ~strcmp(gen_axis{i+1}, gen_axis{i})
            line([0.015 0.985],[yl yl],'Color',[0.45 0.45 0.45],'LineWidth',0.9);
        else
            line([0.015 0.985],[yl yl],'Color',[0.86 0.86 0.86],'LineWidth',0.5);
        end
    end
end
rule(yt(2*nS+0.6), 1.4);

text(0.015, yt(2*nS+1.4), ['X = longitudinal progress error. Path following has no ' ...
     'absolute time reference, so X accumulates - not comparable to trajectory-tracking X.'], ...
     'HorizontalAlignment','left','FontSize',FS-1.5,'Color',[0.4 0.4 0.4], ...
     'Interpreter','none');
if SHOW_EXTRA && any(rate_hz(:) < 9)
    text(0.015, yt(2*nS+2.0), ['Residual runs below 10 Hz predate the solver fix ' ...
         '(109 ms solve). Only Hockenheim v=14 has matched control rates.'], ...
         'HorizontalAlignment','left','FontSize',FS-1.5,'Color',[0.4 0.4 0.4], ...
         'Interpreter','none');
end
