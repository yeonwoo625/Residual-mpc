% 급코너에서의 참조 경로 vs 주행 궤적 - nominal MPC / residual MPC
%
% 기본은 **한 칸**만 그린다: 56 t, 경로 좌표 (s, n).
% 상단 설정으로 적재와 표시 내용을 바꾼다.
%
%   SHOW = 'n'     경로 좌표 (s, n) 한 칸        <- 기본. 차이를 읽는 그림
%   SHOW = 'map'   전역 좌표 (X, Y) 한 칸        <- 코너 형상
%   SHOW = 'both'  둘 다 (1 x 2)
%   MASS_T = 56    32 또는 56
%
% 경로 좌표가 왜 정직한가. 가로축 s(진행 거리)와 세로축 n(횡편차)은 서로 다른
% 물리량이라 눈금 간격이 달라도 된다(시간-속도 그래프와 같은 이치). 그래서 축
% 값을 **실제 미터**로 두고도 1 m 편차가 칸을 가득 채운다. 참조 경로는 n=0 인
% 가로 직선이 된다. n > 0 = 진행방향 기준 왼쪽.
%
% 지도(X, Y)는 두 축이 같은 물리량이라 axis equal 을 지켜야 하고, 그래서
% 2,546 m 트랙 위의 1 m 편차는 창 폭의 1% 로 작게 보인다. 지도에서 편차만 K 배
% 늘리는 방법은 한 그림에 두 배율이 섞여(진행 방향 x1, 수직 방향 xK) 어떤 축
% 눈금을 붙여도 한쪽이 틀리므로 쓰지 않는다.
%
% 주행: Q_n=1e-4, v=10, Hockenheim 건조.
%       잔차 주행은 solve 109 ms 시절이라 실제 제어율이 6.5 Hz 다 (nominal 10 Hz).
%
% 데이터: fig_track_compare.mat  (results/track_compare.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_track_compare`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_track_compare.mat')

MASS_T = 56;             % 32 또는 56
CORNER = 2;              % 1 = s 1232 m,  2 = s 1726 m (차이가 가장 큰 코너)
SHOW   = 'n';            % 'n' | 'map' | 'both'
HALF   = zoom_half;      % 창 반폭 [m]

REF  = [0.72 0.72 0.72];
NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];

mi = find(mass == MASS_T, 1);   % i 는 허수 단위와 겹치므로 쓰지 않는다
if isempty(mi), error('MASS_T 는 %d 또는 %d 여야 한다', mass(1), mass(2)); end
zs = zoom_s(CORNER);
cx = zoom_xy(CORNER,1);  cy = zoom_xy(CORNER,2);

%% ---- 수치 표 (명령창) — 모든 조건을 출력한다 ----
fprintf('\n=== Corner window comparison (window = s +- %.0f m) ===\n', HALF);
fprintf('%7s %5s %14s %12s %12s\n','corner','mass','controller','RMS |n| [m]','max |n| [m]');
for z = 1:numel(zoom_s)
    for a = 1:numel(mass)
        for k = 1:2
            fprintf('%7d %4dt %14s %12.3f %12.3f\n', z, mass(a), variant{k}, ...
                    corner_stat(a,k,z,1), corner_stat(a,k,z,2));
        end
    end
end
fprintf(['\nWhole-lap corner RMS (|kappa| >= 0.02): ' ...
         'nominal %.3f / %.3f,  residual %.3f / %.3f  (32t / 56t)\n'], ...
        met(1,1,2), met(2,1,2), met(1,2,2), met(2,2,2));
fprintf('Residual runs were at 6.5 Hz (pre solver fix), nominal at 10 Hz.\n\n');

%% ---- 그림 ----
xn = traj_x{mi,1};  yn = traj_y{mi,1};   sn = traj_s{mi,1};  nn = traj_n{mi,1};
xr = traj_x{mi,2};  yr = traj_y{mi,2};   sr = traj_s{mi,2};  nr = traj_n{mi,2};

switch lower(SHOW)
    case 'n',    panels = {'n'};            W = 620;
    case 'map',  panels = {'map'};          W = 620;
    otherwise,   panels = {'map','n'};      W = 1120;
end
figure('Color','w','Position',[80 80 W 520])

for pIdx = 1:numel(panels)
    if numel(panels) > 1, subplot(1,2,pIdx); end
    hold on

    if strcmp(panels{pIdx}, 'map')
        % --- 전역 좌표 (지도). 축 비율 1:1 필수 ---
        box on; set(gca,'FontSize',11,'TickDir','out')
        p0 = plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 4);
        p1 = plot(xn, yn, '-', 'Color', NOM, 'LineWidth', 1.6);
        p2 = plot(xr, yr, '-', 'Color', RES, 'LineWidth', 1.6);
        axis equal
        xlim([cx-HALF cx+HALF]); ylim([cy-HALF cy+HALF]);
        legend([p0 p1 p2], {'reference path','Nominal MPC','Residual MPC'}, ...
               'Location','southoutside','Orientation','horizontal', ...
               'Box','off','FontSize',10);
        xlabel('X  [m]'); ylabel('Y  [m]')
        title(sprintf('%d t  -  corner %d,  s = %.0f m', MASS_T, CORNER, zs), ...
              'FontSize',12,'FontWeight','normal')
    else
        % --- 경로 좌표 (s, n). 축 값은 실제 미터 ---
        box off
        set(gca,'FontSize',11,'TickDir','out','XGrid','on','YGrid','on', ...
                'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
        wn = abs(sn - zs) <= HALF;   wr = abs(sr - zs) <= HALF;
        nv = [nn(wn), nr(wr)];
        ylo = min(nv);  yhi = max(nv);
        pad = max((yhi - ylo) * 0.28, 0.05);
        ylo = min(ylo, 0) - pad;   yhi = max(yhi, 0) + pad;

        % 급코너 구간(|kappa| >= 0.02, R <= 50 m) 음영
        wk = (ref_s >= zs-HALF) & (ref_s <= zs+HALF) & (abs(ref_kappa) >= 0.02);
        if any(wk)
            k1 = min(ref_s(wk));  k2 = max(ref_s(wk));
            patch([k1 k2 k2 k1], [ylo ylo yhi yhi], [0.94 0.94 0.94], ...
                  'EdgeColor','none');
            text((k1+k2)/2, ylo + (yhi-ylo)*0.94, 'sharp corner', ...
                 'HorizontalAlignment','center','FontSize',9,'Color',[0.55 0.55 0.55]);
        end

        q0 = plot([zs-HALF zs+HALF], [0 0], '-', 'Color', REF, 'LineWidth', 3);
        q1 = plot(sn(wn), nn(wn), '-', 'Color', NOM, 'LineWidth', 2);
        q2 = plot(sr(wr), nr(wr), '-', 'Color', RES, 'LineWidth', 2);
        legend([q0 q1 q2], {'reference path  (n = 0)','Nominal MPC','Residual MPC'}, ...
               'Location','southoutside','Orientation','horizontal', ...
               'Box','off','FontSize',10);
        text(zs-HALF*0.95, ylo + (yhi-ylo)*0.10, ...
             sprintf('RMS  %.3f  vs  %.3f m      max  %.3f  vs  %.3f m', ...
                     corner_stat(mi,1,CORNER,1), corner_stat(mi,2,CORNER,1), ...
                     corner_stat(mi,1,CORNER,2), corner_stat(mi,2,CORNER,2)), ...
             'FontSize',9.5,'Color',[0.3 0.3 0.3]);
        xlim([zs-HALF zs+HALF]); ylim([ylo yhi])
        xlabel('Path distance  s  [m]')
        ylabel('Lateral deviation  n  [m]')
        title(sprintf('%d t  -  lateral deviation  (corner %d, s = %.0f m)', ...
              MASS_T, CORNER, zs), 'FontSize',12,'FontWeight','normal')
    end
end
