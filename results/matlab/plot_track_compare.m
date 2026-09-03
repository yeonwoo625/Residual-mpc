% 급코너에서의 참조 경로 vs 주행 궤적 - nominal MPC / residual MPC, 32 t 와 56 t
%
% 트랙 한 바퀴가 2,546 m 인데 횡오차는 최대 1.1 m 라 전체 트랙을 그리면 세 선이
% 겹쳐 보인다. 그래서 곡률이 가장 큰 코너만 확대해 비교한다.
%
%   1번 칸  전역 좌표 (X, Y), 실제 스케일 - 코너 형상과 주행 라인
%   2번 칸  같은 전역 좌표인데 **횡편차만 MAG 배 확대** - 곡선을 유지한 채 차이를 본다
%   3번 칸  횡오차 |n| vs 경로거리 s - 차이가 정량적으로 읽힌다
%
% 실제 스케일에서 1 m 편차는 110 m 창의 1% 라 1번 칸에서는 세 선이 거의 겹친다.
% 2번 칸은 경로에 수직인 방향으로만 n 을 MAG 배 늘려 다시 그린 것이다.
%     X = Xr(s) - sin(psi) * (n * MAG),   Y = Yr(s) + cos(psi) * (n * MAG)
% 도로의 곡선 형상은 그대로 두고 편차만 키우므로 '어느 쪽으로 얼마나 밀리는지'가
% 보인다.
%
% ** 축 단위에 대하여 ** X/Y 축 격자는 실제 미터가 맞다 - 참조 경로(회색)는 제
% 위치에 그려진다. 왜곡된 것은 **경로에 수직인 방향뿐**이라, 축 단위를 [m] 이
% 아닌 다른 것으로 바꿔 쓸 수 없다(진행 방향은 실제 스케일이므로 균일 배율이
% 아니다). 그래서 축은 [m] 로 두고, 화면상 MAG 미터가 실제 횡편차 1 m 임을
% 보여주는 **왜곡 눈금자**를 칸 안에 그린다. 지질 단면도의 수직 과장 표기와 같은
% 방식이다. 정량 비교는 3번 칸(실제 값)으로 한다.
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
MAG    = 20;       % 2번 칸의 횡편차 확대 배율 (1 = 확대 없음)
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
figure('Color','w','Position',[40 40 1400 700])
for i = 1:nM
    xn = traj_x{i,1};  yn = traj_y{i,1};   sn = traj_s{i,1};  nn = traj_n{i,1};
    xr = traj_x{i,2};  yr = traj_y{i,2};   sr = traj_s{i,2};  nr = traj_n{i,2};

    % --- (1) 전역 좌표 확대, 실제 스케일 ---
    subplot(nM,3,(i-1)*3+1); hold on; box on
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

    % --- (2) 같은 좌표, 횡편차만 MAG 배 ---
    subplot(nM,3,(i-1)*3+2); hold on; box on
    set(gca,'FontSize',10,'TickDir','out')
    % (s, n) -> 전역 좌표, 횡편차만 MAG 배. 경로 형상은 그대로 둔다.
    % ref_psi 는 저장 시점에 unwrap 되어 있다. 감긴(±pi) 상태로 interp1 하면
    % 감김 지점(s=1373/1711/2181 m)에서 방향이 한 바퀴 돌아 궤적이 뾰족하게
    % 튄다 - 특히 1711 m 는 코너 2 창 안이라 ×20 에서 20 m 도약으로 보인다.
    smn = mod(sn(:), s_total);   smr = mod(sr(:), s_total);
    pxn = interp1(ref_s, ref_x,   smn, 'linear', 'extrap');
    pyn = interp1(ref_s, ref_y,   smn, 'linear', 'extrap');
    psn = interp1(ref_s, ref_psi, smn, 'linear', 'extrap');
    pxr = interp1(ref_s, ref_x,   smr, 'linear', 'extrap');
    pyr = interp1(ref_s, ref_y,   smr, 'linear', 'extrap');
    psr = interp1(ref_s, ref_psi, smr, 'linear', 'extrap');
    xnm = pxn - sin(psn) .* (MAG * nn(:));   ynm = pyn + cos(psn) .* (MAG * nn(:));
    xrm = pxr - sin(psr) .* (MAG * nr(:));   yrm = pyr + cos(psr) .* (MAG * nr(:));
    plot(ref_x, ref_y, '-', 'Color', REF, 'LineWidth', 5);
    plot(xnm, ynm, '-', 'Color', NOM, 'LineWidth', 1.8);
    plot(xrm, yrm, '-', 'Color', RES, 'LineWidth', 1.8);
    axis equal
    xlim([cx-HALF cx+HALF]); ylim([cy-HALF cy+HALF]);
    % 왜곡 눈금자: 화면상 MAG 미터가 실제 횡편차 1 m 임을 보여준다.
    % 축 격자는 실제 미터(참조 경로는 제 위치)이고, 왜곡된 것은 경로에 수직인
    % 방향뿐이라 축 단위를 바꿔 쓸 수 없다. 그래서 눈금자로 표기한다.
    bx = cx - HALF*0.86;  by = cy - HALF*0.86;
    plot([bx bx], [by by+MAG], '-', 'Color', 'k', 'LineWidth', 2.2);
    plot([bx-HALF*0.03 bx+HALF*0.03], [by by], '-', 'Color','k', 'LineWidth', 1.4);
    plot([bx-HALF*0.03 bx+HALF*0.03], [by+MAG by+MAG], '-', 'Color','k', 'LineWidth', 1.4);
    text(bx+HALF*0.06, by+MAG/2, {'1 m','actual'}, ...
         'FontSize',9,'Color','k','VerticalAlignment','middle');
    xlabel('X  [m]   (axes are true scale)')
    title(sprintf('%d t  -  lateral deviation exaggerated \\times%d', ...
          mass(i), MAG), 'FontSize',11,'FontWeight','normal')

    % --- (3) 횡오차 |n| vs s ---
    subplot(nM,3,(i-1)*3+3); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
            'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
    wn = abs(sn - zs) <= HALF;   wr = abs(sr - zs) <= HALF;
    plot([zs-HALF zs+HALF], [0 0], '-', 'Color', REF, 'LineWidth', 3);
    plot(sn(wn), abs(nn(wn)), '-', 'Color', NOM, 'LineWidth', 1.8);
    plot(sr(wr), abs(nr(wr)), '-', 'Color', RES, 'LineWidth', 1.8);
    % nn/nr 은 셀에서 행 벡터로 나오고 창 안 점 개수도 서로 달라, 세로로 붙이면
    % 차원 불일치가 난다. 각자 최대값을 먼저 구해 스칼라끼리 비교한다.
    ymax = max([max(abs(nn(wn))), max(abs(nr(wr)))]) * 1.35;
    if isempty(ymax) || ~isfinite(ymax) || ymax <= 0
        ymax = 1;                      % 창 안에 점이 없을 때의 안전값
    end
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
