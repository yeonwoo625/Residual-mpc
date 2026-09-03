% 급코너에서의 참조 경로 vs 주행 궤적 - nominal MPC / residual MPC, 32 t 와 56 t
%
%   왼쪽 칸  전역 좌표 (X, Y). 지도. 코너 형상과 주행 라인.
%   오른쪽 칸 경로 좌표 (s, n). 참조 경로가 n=0 인 직선이 된다. 차이를 읽는 칸.
%
% 두 칸 모두 **축 값이 실제 미터**다. 과장 배율을 쓰지 않는다.
%
% 왜 오른쪽 칸에서는 차이가 크게 보이는가. 지도(왼쪽)는 X, Y 가 같은 물리량이라
% 축 비율을 반드시 1:1 로 둬야 하고(axis equal), 그래서 2,546 m 트랙 위의 1 m
% 편차는 창 폭의 1% 로 작게 보인다. 반면 경로 좌표(오른쪽)는 가로가 진행 거리 s,
% 세로가 횡편차 n 으로 **서로 다른 물리량**이라 눈금 간격이 달라도 된다. 시간-속도
% 그래프에서 축 비율을 따지지 않는 것과 같다. 그래서 축 값을 실제 미터로 두고도
% 궤적 차이를 크게 보여줄 수 있다 - 왜곡이 아니다.
%
% (참고: 지도에서 편차만 K 배 늘려 그리는 방법도 있으나, 그러면 한 그림에 두
%  배율이 섞여 축 눈금을 정직하게 붙일 수 없다. 진행 방향은 실제 스케일이고
%  수직 방향만 K 배이므로, 눈금을 K 로 나누면 편차는 맞고 코너 반경이 K 배
%  틀린다. 그래서 쓰지 않는다.)
%
% n 은 부호를 살렸다. n > 0 = 진행방향 기준 왼쪽. 이 코너에서는 두 제어기 모두
% n < 0 (바깥쪽으로 밀림) 이다. 급코너 구간(|kappa| >= 0.02)은 음영으로 표시한다.
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

CORNER = 2;              % 1 = s 1232 m,  2 = s 1726 m (차이가 가장 큰 코너)
HALF   = zoom_half;      % 창 반폭 [m]. 줄이면 확대된다 (예: 25)

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

%% ---- 그림: 행 = 적재, 열 = [XY 실제 스케일, |n| vs s] ----
figure('Color','w','Position',[60 60 1080 700])
for i = 1:nM
    xn = traj_x{i,1};  yn = traj_y{i,1};   sn = traj_s{i,1};  nn = traj_n{i,1};
    xr = traj_x{i,2};  yr = traj_y{i,2};   sr = traj_s{i,2};  nr = traj_n{i,2};

    % --- (좌) 전역 좌표, 실제 스케일 ---
    subplot(nM,2,(i-1)*2+1); hold on; box on
    set(gca,'FontSize',10,'TickDir','out')
    p0 = plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 4);
    p1 = plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 1.6);
    p2 = plot(xr, yr, '-', 'Color', RES, 'LineWidth', 1.6);
    axis equal
    xlim([cx-HALF cx+HALF]); ylim([cy-HALF cy+HALF]);
    if i == 1
        legend([p0 p1 p2], {'reference path','Nominal MPC','Residual MPC'}, ...
               'Location','southoutside','Orientation','horizontal', ...
               'Box','off','FontSize',9);
    end
    xlabel('X  [m]'); ylabel('Y  [m]')
    title(sprintf('%d t  -  corner %d,  s = %.0f m', mass(i), CORNER, zs), ...
          'FontSize',11,'FontWeight','normal')

    % --- (우) 경로 좌표계 (s, n) - 참조 경로가 n=0 인 직선이 된다 ---
    % 가로축 s(진행 거리) 와 세로축 n(횡편차) 은 서로 다른 물리량이라 눈금 간격이
    % 달라도 왜곡이 아니다. 그래서 축 값은 실제 미터 그대로 두고도 궤적 차이를
    % 크게 보여줄 수 있다. 부호를 살려 어느 쪽으로 벗어나는지도 보인다
    % (n > 0 = 진행방향 기준 왼쪽).
    subplot(nM,2,(i-1)*2+2); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
    wn = abs(sn - zs) <= HALF;   wr = abs(sr - zs) <= HALF;
    nv = [nn(wn), nr(wr)];
    ylo = min(nv);  yhi = max(nv);
    pad = max((yhi - ylo) * 0.30, 0.05);
    ylo = min(ylo, 0) - pad;   yhi = max(yhi, 0) + pad;

    % 급코너 구간(|kappa| >= 0.02, R <= 50 m) 음영 - 어디가 코너인지 표시
    wk = (ref_s >= zs-HALF) & (ref_s <= zs+HALF) & (abs(ref_kappa) >= 0.02);
    if any(wk)
        k1 = min(ref_s(wk));  k2 = max(ref_s(wk));
        patch([k1 k2 k2 k1], [ylo ylo yhi yhi], [0.94 0.94 0.94], ...
              'EdgeColor','none');
        text((k1+k2)/2, yhi*0.90 + ylo*0.10, 'sharp corner', ...
             'HorizontalAlignment','center','FontSize',8.5,'Color',[0.55 0.55 0.55]);
    end

    q0 = plot([zs-HALF zs+HALF], [0 0], '-', 'Color', REF, 'LineWidth', 3);
    q1 = plot(sn(wn), nn(wn), '-', 'Color', NOM, 'LineWidth', 1.8);
    q2 = plot(sr(wr), nr(wr), '-', 'Color', RES, 'LineWidth', 1.8);
    if i == 1
        legend([q0 q1 q2], {'reference path  (n = 0)','Nominal MPC','Residual MPC'}, ...
               'Location','southoutside','Orientation','horizontal', ...
               'Box','off','FontSize',9);
    end
    text(zs-HALF*0.95, ylo + (yhi-ylo)*0.10, ...
         sprintf('RMS  %.3f  vs  %.3f m   |   max  %.3f  vs  %.3f m', ...
                 corner_stat(i,1,CORNER,1), corner_stat(i,2,CORNER,1), ...
                 corner_stat(i,1,CORNER,2), corner_stat(i,2,CORNER,2)), ...
         'FontSize',9,'Color',[0.3 0.3 0.3]);
    xlim([zs-HALF zs+HALF]); ylim([ylo yhi])
    xlabel('Path distance  s  [m]')
    ylabel('Lateral deviation  n  [m]')
    title(sprintf('%d t  -  path frame (axes both true metres)', mass(i)), ...
          'FontSize',11,'FontWeight','normal')
end
