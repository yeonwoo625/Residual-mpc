% 제어 입출력 - nominal vs residual, 세 조건
%
% MPC 가 차량에 보내는 것은 스로틀 D 와 조향각 delta 다(속도가 아니다). 상태
% [s,n,alpha,v,D,delta] 중 D 와 delta 가 명령이고, 그 변화율이 최적화 변수다.
% 속도와 가속도는 명령이 아니라 스로틀의 결과(응답)다.
%
%   신호 1  스로틀 D          [-1, 1].  D>0 구동, D<0 제동
%   신호 2  조향각 delta      제약 +-30 deg (점선)
%   신호 3  조향각속도        제어 주기로 나눈 값
%   신호 4  속도 v            응답
%   신호 5  가속도 a = dv/dt  응답. 제어 주기로 나눈 값
%
% ** 제어율 주의 ** 잔차 주행 중 저마찰(6.4 Hz)과 v=12(6.5 Hz)는 solve 가 109 ms
% 이던 시절이라 10 Hz 를 못 지켰다. 조향각속도와 가속도는 dt 로 나누므로 이 차이가
% 값에 직접 들어간다. 고속(Highway v=20) 조건만 양쪽 10 Hz 로 공정한 비교다.
%
% 데이터: fig_control_inputs.mat  (results/control_inputs.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_control_inputs`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_control_inputs.mat')

CASE    = 0;        % 0 = 세 조건 모두, 1 = 저마찰, 2 = 고속, 3 = 기준
XAXIS   = 'both';   % 'dist' | 'time' | 'both'
SIGNALS = 'four';   % 'four' = 조향각/조향각속도/속도/가속도 를 한 창에
                    % 'all'  = 창 2개 (명령 3개 + 응답 2개)
%
% 어느 축을 쓸지
%   dist  같은 지점을 나란히 비교할 때. 두 주행의 속도가 달라도 위치가 맞는다.
%   time  속도 상승·유지 양상을 볼 때. 제어율이 달라도 초 단위는 같으나
%         같은 t 가 같은 지점을 뜻하지는 않는다.

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];

if CASE == 0, rows = 1:numel(case_); else rows = CASE; end
nR = numel(rows);

% 신호 정의: {데이터, y라벨, 제목}
SIG = {throttle,       'Throttle  D  [-]',        'throttle'; ...
       delta_deg,      'Steering  \delta  [deg]', 'steering angle'; ...
       delta_rate_dps, 'Steering rate  [deg/s]',  'steering rate'; ...
       speed,          'Speed  v  [m/s]',         'speed'; ...
       accel,          'Acceleration  [m/s^2]',   'acceleration'};

if strcmpi(SIGNALS, 'four')
    GROUPS = {[2 3 4 5]};
else
    GROUPS = {[1 2 3], [4 5]};
end
if     strcmpi(XAXIS,'time'), AXES = {'time'};
elseif strcmpi(XAXIS,'dist'), AXES = {'dist'};
else                          AXES = {'dist','time'};
end

%% ---- 수치 표 (명령창) ----
fprintf('\n=== Control commands ===\n');
fprintf('%-28s %14s %6s %8s %6s %6s %10s %9s %9s\n', 'case', 'controller', ...
        'Hz', 'D mean', 'full', 'brake', 'max|d|deg', 'dd rms', 'dd max');
for i = 1:numel(case_)
    for k = 1:2
        if k == 1, nm = case_{i}; else nm = ''; end
        fprintf('%-28s %14s %6.1f %8.3f %5.0f%% %5.0f%% %10.1f %9.2f %9.1f\n', ...
                nm, variant{k}, met(i,k,1), met(i,k,2), met(i,k,3), ...
                met(i,k,4), met(i,k,5), met(i,k,6), met(i,k,7));
    end
end
fprintf('\n=== Vehicle response ===\n');
fprintf('%-28s %14s %9s %9s %10s %10s\n', 'case', 'controller', ...
        'v mean', 'v max', 'a mean', 'max|a|');
for i = 1:numel(case_)
    for k = 1:2
        if k == 1, nm = case_{i}; else nm = ''; end
        fprintf('%-28s %14s %9.2f %9.2f %+10.3f %10.2f\n', nm, variant{k}, ...
                met(i,k,8), met(i,k,9), met(i,k,10), met(i,k,11));
    end
end
fprintf(['\nSteering rate and acceleration divide by the control period. Residual ' ...
         'runs at 6.4-6.5 Hz\n(pre solver fix) except the high-speed case, where ' ...
         'both are 10 Hz.\n\n']);

%% ---- 그림 ----
for ax = 1:numel(AXES)
    if strcmpi(AXES{ax}, 'time')
        XV = t;  XLAB = 'Time  [s]';   tag = 'vs time';
    else
        XV = s;  XLAB = 'Path distance  s  [m]';  tag = 'vs distance';
    end

    for gi = 1:numel(GROUPS)
        g  = GROUPS{gi};
        nC = numel(g);
        figure('Color','w','Position',[40 40 min(340*nC,1400) 290*nR])
        for r = 1:nR
            i = rows(r);
            xn = XV{i,1};  xr = XV{i,2};
            xmax = max([max(xn) max(xr)]);
            for c = 1:nC
                sid = g(c);
                dat = SIG{sid,1};
                subplot(nR,nC,(r-1)*nC + c); hold on; box off
                set(gca,'FontSize',9,'TickDir','out','XGrid','on','YGrid','on', ...
                        'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')

                if sid ~= 4        % 속도 말고는 0 기준선을 그린다
                    plot([0 xmax],[0 0],'-','Color',[0.75 0.75 0.75],'LineWidth',0.8);
                end
                if sid == 2        % 조향각 제약선
                    plot([0 xmax],[ delta_max_deg  delta_max_deg],'--', ...
                         'Color',GREY,'LineWidth',1.2);
                    plot([0 xmax],[-delta_max_deg -delta_max_deg],'--', ...
                         'Color',GREY,'LineWidth',1.2);
                end
                h1 = plot(xn, dat{i,1}, '-', 'Color', NOM, 'LineWidth', 1.0);
                h2 = plot(xr, dat{i,2}, '-', 'Color', RES, 'LineWidth', 1.0);
                if r == 1 && c == 1
                    legend([h1 h2], variant, 'Location','best','Box','off','FontSize',8);
                end

                % y 범위와 요약 텍스트
                switch sid
                    case 1
                        ylim([-0.15 1.15]);
                        txt = sprintf('full  %.0f%%  vs  %.0f%%', met(i,1,3), met(i,2,3));
                    case 2
                        L = delta_max_deg*1.15; ylim([-L L]);
                        txt = sprintf('max  %.1f\\circ  vs  %.1f\\circ', met(i,1,5), met(i,2,5));
                    case 3
                        L = max([met(i,1,7) met(i,2,7)])*1.15; ylim([-L L]);
                        txt = sprintf('RMS  %.2f  vs  %.2f\\circ/s', met(i,1,6), met(i,2,6));
                    case 4
                        L = max([met(i,1,9) met(i,2,9)])*1.12; ylim([0 L]);
                        txt = sprintf('mean  %.2f  vs  %.2f m/s', met(i,1,8), met(i,2,8));
                    case 5
                        L = max([met(i,1,11) met(i,2,11)])*1.15; ylim([-L L]);
                        txt = sprintf('max |a|  %.2f  vs  %.2f', met(i,1,11), met(i,2,11));
                end
                yl = ylim;
                text(xmax*0.02, yl(1)+(yl(2)-yl(1))*0.90, txt, ...
                     'FontSize',8,'Color',[0.3 0.3 0.3]);
                if c == nC && met(i,2,1) < 9
                    text(xmax*0.98, yl(1)+(yl(2)-yl(1))*0.08, ...
                         sprintf('%.1f / %.1f Hz', met(i,1,1), met(i,2,1)), ...
                         'HorizontalAlignment','right','FontSize',8,'Color',[0.6 0.3 0.3]);
                end

                xlim([0 xmax]);
                ylabel(SIG{sid,2});
                if r == nR, xlabel(XLAB); end
                if c == 1
                    title(sprintf('%s   -   %s   (%s)', case_{i}, SIG{sid,3}, tag), ...
                          'FontSize',10,'FontWeight','normal')
                else
                    title(SIG{sid,3}, 'FontSize',10,'FontWeight','normal')
                end
            end
        end
    end
end
