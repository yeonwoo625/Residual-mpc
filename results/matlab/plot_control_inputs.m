% 제어 입출력 - nominal vs residual, 세 조건
%
% MPC 가 차량에 보내는 것은 스로틀 D 와 조향각 delta 다(속도가 아니다).
% 상태 [s,n,alpha,v,D,delta] 중 D 와 delta 가 명령이고, 그 변화율이 최적화
% 변수다. 그래서 조향각속도(delta 의 시간 미분)까지 함께 본다.
%
% 창이 두 개 뜬다.
%   Figure 1  제어 명령    스로틀 D / 조향각 delta / 조향각속도
%   Figure 2  차량 응답    속도 v / 가속도 a
%
% 속도는 명령이 아니라 스로틀의 결과다. 명령과 응답을 나란히 봐야 제어기가
% 무엇을 했고 차량이 어떻게 반응했는지 읽힌다.
%
%   Fig1 1열  스로틀 D          [-1, 1].  D>0 구동, D<0 제동
%   Fig1 2열  조향각 delta      제약 +-30 deg (점선)
%   Fig1 3열  조향각속도        제어 주기로 나눈 값
%   Fig2 1열  속도 v
%   Fig2 2열  가속도 a = dv/dt  (제어 주기로 나눈 값)
%
% 가로축은 경로거리 s 다. 제어율이 조건마다 달라 시간축으로는 나란히 놓을 수 없다.
%
% ** 제어율 주의 ** 잔차 주행 중 저마찰(6.4 Hz)과 v=12(6.5 Hz)는 solve 가 109 ms
% 이던 시절이라 10 Hz 를 못 지켰다. 조향각속도는 dt 로 나누므로 이 차이가 값에
% 직접 들어간다. 고속(Highway v=20) 조건만 양쪽 10 Hz 로 공정한 비교다.
%
% CASE 로 한 조건만 볼 수 있다 (0 = 전부).
%
% 데이터: fig_control_inputs.mat  (results/control_inputs.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_control_inputs`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_control_inputs.mat')

CASE  = 0;          % 0 = 세 조건 모두, 1 = 저마찰, 2 = 고속, 3 = 기준
XAXIS = 'dist';     % 'dist' = 경로거리 s [m]  |  'time' = 시간 t [s]
%
% 어느 축을 쓸지
%   dist  같은 지점을 나란히 비교할 때. 두 주행의 속도가 달라도 위치가 맞는다.
%   time  속도가 어떻게 올라가고 유지되는지 볼 때. 제어율이 달라도 초 단위는
%         같으나, 같은 t 가 같은 지점을 뜻하지는 않는다.

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];

if CASE == 0, rows = 1:numel(case_); else rows = CASE; end
nR = numel(rows);

if strcmpi(XAXIS, 'time')
    XV = t;    XLAB = 'Time  [s]';
else
    XV = s;    XLAB = 'Path distance  s  [m]';
end

%% ---- 수치 표 (명령창) ----
fprintf('\n=== Control commands: nominal vs residual ===\n');
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

figure('Color','w','Position',[40 40 1340 300*nR])
for r = 1:nR
    i = rows(r);
    sn = XV{i,1};  sr = XV{i,2};
    smax = max([max(sn) max(sr)]);

    % --- 스로틀 ---
    subplot(nR,3,(r-1)*3+1); hold on; box off
    set(gca,'FontSize',9,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')
    plot([0 smax],[0 0],'-','Color',[0.75 0.75 0.75],'LineWidth',0.8);
    h1 = plot(sn, throttle{i,1}, '-', 'Color', NOM, 'LineWidth', 1.0);
    h2 = plot(sr, throttle{i,2}, '-', 'Color', RES, 'LineWidth', 1.0);
    if r == 1
        legend([h1 h2], variant, 'Location','southeast','Box','off','FontSize',8);
    end
    xlim([0 smax]); ylim([-0.15 1.15])
    ylabel('Throttle  D  [-]')
    if r == nR, xlabel(XLAB); end
    title(sprintf('%s   -   throttle', case_{i}), 'FontSize',10,'FontWeight','normal')

    % --- 조향각 ---
    subplot(nR,3,(r-1)*3+2); hold on; box off
    set(gca,'FontSize',9,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')
    plot([0 smax],[ delta_max_deg  delta_max_deg],'--','Color',GREY,'LineWidth',1.2);
    plot([0 smax],[-delta_max_deg -delta_max_deg],'--','Color',GREY,'LineWidth',1.2);
    plot(sn, delta_deg{i,1}, '-', 'Color', NOM, 'LineWidth', 1.0);
    plot(sr, delta_deg{i,2}, '-', 'Color', RES, 'LineWidth', 1.0);
    text(smax*0.98, delta_max_deg*0.86, sprintf('limit  \\pm%.0f\\circ', delta_max_deg), ...
         'HorizontalAlignment','right','FontSize',8,'Color',GREY);
    text(smax*0.02, -delta_max_deg*0.80, ...
         sprintf('max  %.1f\\circ  vs  %.1f\\circ', met(i,1,5), met(i,2,5)), ...
         'FontSize',8.5,'Color',[0.3 0.3 0.3]);
    xlim([0 smax]); ylim([-delta_max_deg*1.15 delta_max_deg*1.15])
    ylabel('Steering  \delta  [deg]')
    if r == nR, xlabel(XLAB); end
    title('steering angle', 'FontSize',10,'FontWeight','normal')

    % --- 조향각속도 ---
    subplot(nR,3,(r-1)*3+3); hold on; box off
    set(gca,'FontSize',9,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')
    plot([0 smax],[0 0],'-','Color',[0.75 0.75 0.75],'LineWidth',0.8);
    plot(sn, delta_rate_dps{i,1}, '-', 'Color', NOM, 'LineWidth', 0.9);
    plot(sr, delta_rate_dps{i,2}, '-', 'Color', RES, 'LineWidth', 0.9);
    L = max([met(i,1,7) met(i,2,7)]) * 1.15;
    text(smax*0.02, L*0.82, sprintf('RMS  %.2f  vs  %.2f\\circ/s', ...
         met(i,1,6), met(i,2,6)), 'FontSize',8.5,'Color',[0.3 0.3 0.3]);
    if met(i,2,1) < 9
        text(smax*0.98, -L*0.82, sprintf('rates: %.1f / %.1f Hz', met(i,1,1), met(i,2,1)), ...
             'HorizontalAlignment','right','FontSize',8,'Color',[0.6 0.3 0.3]);
    end
    xlim([0 smax]); ylim([-L L])
    ylabel('Steering rate  [deg/s]')
    if r == nR, xlabel(XLAB); end
    title('steering rate', 'FontSize',10,'FontWeight','normal')
end

%% ==== Figure 2: 차량 응답 (속도, 가속도) ====
figure('Color','w','Position',[80 60 940 300*nR])
for r = 1:nR
    i = rows(r);
    sn = XV{i,1};  sr = XV{i,2};
    smax = max([max(sn) max(sr)]);

    % --- 속도 ---
    subplot(nR,2,(r-1)*2+1); hold on; box off
    set(gca,'FontSize',9,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')
    h1 = plot(sn, speed{i,1}, '-', 'Color', NOM, 'LineWidth', 1.2);
    h2 = plot(sr, speed{i,2}, '-', 'Color', RES, 'LineWidth', 1.2);
    if r == 1
        legend([h1 h2], variant, 'Location','southeast','Box','off','FontSize',8);
    end
    text(smax*0.02, max([met(i,1,9) met(i,2,9)])*0.20, ...
         sprintf('mean  %.2f  vs  %.2f m/s', met(i,1,8), met(i,2,8)), ...
         'FontSize',8.5,'Color',[0.3 0.3 0.3]);
    xlim([0 smax]); ylim([0 max([met(i,1,9) met(i,2,9)])*1.12])
    ylabel('Speed  v  [m/s]')
    if r == nR, xlabel(XLAB); end
    title(sprintf('%s   -   speed', case_{i}), 'FontSize',10,'FontWeight','normal')

    % --- 가속도 ---
    subplot(nR,2,(r-1)*2+2); hold on; box off
    set(gca,'FontSize',9,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')
    plot([0 smax],[0 0],'-','Color',[0.75 0.75 0.75],'LineWidth',0.8);
    plot(sn, accel{i,1}, '-', 'Color', NOM, 'LineWidth', 0.9);
    plot(sr, accel{i,2}, '-', 'Color', RES, 'LineWidth', 0.9);
    L = max([met(i,1,11) met(i,2,11)]) * 1.15;
    text(smax*0.02, L*0.82, sprintf('max |a|  %.2f  vs  %.2f m/s^2', ...
         met(i,1,11), met(i,2,11)), 'FontSize',8.5,'Color',[0.3 0.3 0.3]);
    if met(i,2,1) < 9
        text(smax*0.98, -L*0.82, sprintf('rates: %.1f / %.1f Hz', met(i,1,1), met(i,2,1)), ...
             'HorizontalAlignment','right','FontSize',8,'Color',[0.6 0.3 0.3]);
    end
    xlim([0 smax]); ylim([-L L])
    ylabel('Acceleration  [m/s^2]')
    if r == nR, xlabel(XLAB); end
    title('acceleration', 'FontSize',10,'FontWeight','normal')
end
