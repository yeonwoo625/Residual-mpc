% 급코너에서의 참조 경로 vs 주행 궤적 - nominal MPC / residual MPC, 32 t 와 56 t
%
% 트랙 한 바퀴가 2,546 m 인데 횡오차는 최대 1.1 m 라 전체 트랙을 그리면 세 선이
% 겹쳐 보인다. 그래서 곡률이 가장 큰 코너만 확대해 비교한다.
%
%   왼쪽 칸  전역 좌표 (X, Y) - 실제 스케일. 경로를 어떻게 도는지 보인다.
%   오른쪽 칸 횡오차 |n| vs 경로거리 s - 차이가 정량적으로 읽힌다.
%
% 실제 스케일에서 1 m 편차는 110 m 창의 1% 라 왼쪽 칸에서도 크게 보이지 않는다.
% **차이를 읽는 칸은 오른쪽이다.** 왼쪽은 코너 형상과 주행 라인을 보여준다.
%
% CORNER 로 두 코너 중 하나를 고른다. 코너 2 (s=1726 m) 에서 차이가 가장 크다.
%
% 주행: Q_n=1e-4, v=10, Hockenheim 건조.
%       잔차 주행은 solve 109 ms 시절이라 실제 제어율이 6.5 Hz 다 (nominal 10 Hz).
%
% 데이터: fig_track_compare.mat  (results/track_compare.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_track_compare`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_track_compare.mat')

CORNER = 2;        % 1 = s 1232 m,  2 = s 1726 m (차이가 가장 큰 코너)
HALF   = zoom_half;

REF  = [0.72 0.72 0.72];
NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];

nM = numel(mass);
zs = zoom_s(CORNER);
cx = zoom_xy(CORNER,1);  cy = zoom_xy(CORNER,2);

%% ---- 수치 표 (명령창) ----
fprintf('\n=== Corner window comparison (window = s %.0f +- %.0f m) ===\n', zs, HALF);
fprintf('%7s %5s %14s %12s %12s\n','corner','mass','controller','RMS |n| [m]','max |n| [m]');
for z = 1:numel(zoom_s)
    for i = 1:nM
        for k = 1:2
            fprintf('%7d %4dt %14s %12.3f %12.3f\n', z, mass(i), variant{k}, ...
                    corner_stat(i,k,z,1), corner_stat(i,k,z,2));
        end
    end
end
fprintf(['\nWhole-lap corner RMS (|kappa| >= 0.02): ' ...
         'nominal %.3f / %.3f,  residual %.3f / %.3f  (32t / 56t)\n'], ...
        met(1,1,2), met(2,1,2), met(1,2,2), met(2,2,2));
fprintf('Residual runs were at 6.5 Hz (pre solver fix), nominal at 10 Hz.\n\n');

%% ---- 그림: 행 = 적재, 열 = [XY 확대, |n| vs s] ----
figure('Color','w','Position',[60 60 1080 700])
for i = 1:nM
    xn = traj_x{i,1};  yn = traj_y{i,1};   sn = traj_s{i,1};  nn = traj_n{i,1};
    xr = traj_x{i,2};  yr = traj_y{i,2};   sr = traj_s{i,2};  nr = traj_n{i,2};

    % --- (좌) 전역 좌표 확대 ---
    subplot(nM,2,(i-1)*2+1); hold on; box on
    set(gca,'FontSize',10,'TickDir','out')
    p0 = plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 5);
    p1 = plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 1.8);
    p2 = plot(xr, yr, '-', 'Color', RES, 'LineWidth', 1.8);
    axis equal
    xlim([cx-HALF cx+HALF]); ylim([cy-HALF cy+HALF]);
    if i == 1
        legend([p0 p1 p2], {'reference path','Nominal MPC','Residual MPC'}, ...
               'Location','southoutside','Orientation','horizontal', ...
               'Box','off','FontSize',9);
    end
    xlabel('X  [m]'); ylabel('Y  [m]')
    title(sprintf('%d t  -  corner %d,  s = %.0f m  (true scale)', ...
          mass(i), CORNER, zs), 'FontSize',11,'FontWeight','normal')

    % --- (우) 횡오차 |n| vs s ---
    subplot(nM,2,(i-1)*2+2); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
    wn = abs(sn - zs) <= HALF;   wr = abs(sr - zs) <= HALF;
    plot([zs-HALF zs+HALF], [0 0], '-', 'Color', REF, 'LineWidth', 3);
    plot(sn(wn), abs(nn(wn)), '-', 'Color', NOM, 'LineWidth', 1.8);
    plot(sr(wr), abs(nr(wr)), '-', 'Color', RES, 'LineWidth', 1.8);
    ymax = max([abs(nn(wn)); abs(nr(wr))]) * 1.35;
    text(zs-HALF*0.95, ymax*0.93, ...
         sprintf('RMS  %.3f  vs  %.3f m', corner_stat(i,1,CORNER,1), ...
                 corner_stat(i,2,CORNER,1)), 'FontSize',9.5,'Color',[0.3 0.3 0.3]);
    text(zs-HALF*0.95, ymax*0.83, ...
         sprintf('max  %.3f  vs  %.3f m', corner_stat(i,1,CORNER,2), ...
                 corner_stat(i,2,CORNER,2)), 'FontSize',9.5,'Color',[0.3 0.3 0.3]);
    xlim([zs-HALF zs+HALF]); ylim([0 ymax])
    xlabel('Path distance  s  [m]'); ylabel('Lateral error  |n|  [m]')
    title(sprintf('%d t  -  lateral error in the same window', mass(i)), ...
          'FontSize',11,'FontWeight','normal')
end
