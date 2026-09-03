% 참조 경로 vs 주행 궤적 - nominal MPC / residual MPC, 32 t 와 56 t
%
% Frenet (s, n) 을 전역 좌표로 되돌려 기준 경로 위에 겹쳐 그린다.
%
% ** 스케일 주의 ** 트랙 한 바퀴가 2,546 m 인데 횡오차는 최대 1.1 m 다.
% 전체 트랙(왼쪽 칸)에서는 세 선이 겹쳐 보인다. 그래서 가장 급한 코너 두 곳을
% 확대해 함께 그린다(가운데/오른쪽 칸). 확대 창에서만 차이가 눈에 보인다.
%
% 주행: Q_n=1e-4, v=10, Hockenheim 건조.
%       잔차 주행은 solve 109 ms 시절이라 실제 제어율이 6.5 Hz 다 (nominal 10 Hz).
%
% 데이터: fig_track_compare.mat  (results/track_compare.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_track_compare`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_track_compare.mat')

REF  = [0.72 0.72 0.72];
NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];

nM = numel(mass);  nZ = size(zoom_xy,1);

%% ---- 수치 표 (명령창) ----
fprintf('\n=== Reference path vs driven trajectory ===\n');
fprintf('track length %.0f m,  zoom windows at s = %.0f / %.0f m (half %.0f m)\n', ...
        s_total, zoom_s(1), zoom_s(2), zoom_half);
fprintf('%5s %14s %9s %10s %9s %9s\n', ...
        'mass','controller','rate[Hz]','corner|n|','mean|n|','max|n|');
for i = 1:nM
    for k = 1:2
        fprintf('%4dt %14s %9.1f %10.3f %9.3f %9.3f\n', mass(i), variant{k}, ...
                met(i,k,1), met(i,k,2), met(i,k,3), met(i,k,4));
    end
end
fprintf(['corner |n| = RMS over |kappa| >= 0.02 (R <= 50 m).  ' ...
         'Residual runs were at 6.5 Hz (pre solver fix).\n\n']);

%% ---- 그림: 행 = 적재, 열 = [전체 트랙, 확대1, 확대2] ----
figure('Color','w','Position',[50 50 1280 720])
for i = 1:nM
    xn = traj_x{i,1};  yn = traj_y{i,1};      % nominal
    xr = traj_x{i,2};  yr = traj_y{i,2};      % residual

    % --- 전체 트랙 ---
    subplot(nM, nZ+1, (i-1)*(nZ+1) + 1); hold on; box off
    set(gca,'FontSize',9,'TickDir','out')
    p0 = plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 3);
    p1 = plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 0.8);
    p2 = plot(xr, yr, '-', 'Color', RES, 'LineWidth', 0.8);
    for z = 1:nZ                                  % 확대 창 위치 표시
        cx = zoom_xy(z,1); cy = zoom_xy(z,2); h = zoom_half;
        plot([cx-h cx+h cx+h cx-h cx-h], [cy-h cy-h cy+h cy+h cy-h], ...
             '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.1);
        text(cx, cy+h*1.12, sprintf('%d', z), 'HorizontalAlignment','center', ...
             'FontSize',9,'FontWeight','bold','Color',[0.25 0.25 0.25]);
    end
    axis equal; axis tight
    if i == 1
        legend([p0 p1 p2], {'reference path','Nominal MPC','Residual MPC'}, ...
               'Location','best','Box','off','FontSize',8);
    end
    xlabel('X  [m]'); ylabel('Y  [m]')
    title(sprintf('%d t  -  full lap (deviations invisible at this scale)', mass(i)), ...
          'FontSize',10,'FontWeight','normal')

    % --- 확대 창 ---
    for z = 1:nZ
        subplot(nM, nZ+1, (i-1)*(nZ+1) + 1 + z); hold on; box on
        set(gca,'FontSize',9,'TickDir','out')
        plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 4);
        plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 1.6);
        plot(xr, yr, '-', 'Color', RES, 'LineWidth', 1.6);
        cx = zoom_xy(z,1); cy = zoom_xy(z,2); h = zoom_half;
        axis equal
        xlim([cx-h cx+h]); ylim([cy-h cy+h]);
        xlabel('X  [m]')
        title(sprintf('%d t  -  corner %d  (s = %.0f m)', mass(i), z, zoom_s(z)), ...
              'FontSize',10,'FontWeight','normal')
        if z == 1
            text(0.03, 0.06, sprintf('corner |n| RMS:  %.3f  vs  %.3f m', ...
                 met(i,1,2), met(i,2,2)), 'Units','normalized', ...
                 'FontSize',8.5,'Color',[0.3 0.3 0.3]);
        end
    end
end
