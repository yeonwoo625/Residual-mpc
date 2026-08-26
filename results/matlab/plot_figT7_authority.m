function plot_figT7_authority(run)
% figT7 - 잔차 Jacobian 주입이 조향 권한(steering authority)에 미치는 영향
%
%   plot_figT7_authority            % 기본: js05 (lambda=0.5, 발산한 주행)
%   plot_figT7_authority('ff')      % lambda=0 (완주)
%   plot_figT7_authority('js10')    % lambda=1.0 (발산)
%
% 데이터: figT7_authority.mat  (results/matlab/)
%   auth_nominal     = ||d n_N / d delta||,  Jacobian A = df/dx        (nominal)
%   auth_firstorder  = ||d n_N / d delta||,  A = df/dx + dDelta/dx     (first-order)
%   jitter_*         = 직전 스텝 대비 권한 변화율 (|diff|/value)
%   N = 20, dt = 0.1 s, 모델 residual_model_nomass_s0.pt

if nargin < 1, run = 'js05'; end
S = load('figT7_authority.mat');
D = S.(run);

s   = D.s(:);      a0 = D.auth_nominal(:);   a1 = D.auth_firstorder(:);
kap = D.kappa(:);  j0 = D.jitter_nominal(:); j1 = D.jitter_firstorder(:);
corner = abs(kap) >= quantile(abs(kap), 0.80);      % 급코너 = |kappa| 상위 20%

figure('Color','w','Position',[100 100 900 640]);

% ---- (a) 권한 프로파일 -------------------------------------------------
subplot(3,1,[1 2]); hold on; box on
yl = [0 max([a0;a1])*1.1];
shade_corners(s, corner, yl);
plot(s, a0, 'Color',[0.16 0.55 0.36], 'LineWidth',1.6);
plot(s, a1, 'Color',[0.78 0.24 0.20], 'LineWidth',1.6);
ylim(yl); xlim([min(s) max(s)]);
ylabel('조향 권한  ||\partial n_N / \partial\delta||');
legend({'급코너 (R \leq 50 m)','nominal (\lambda = 0)', ...
        'first-order (\lambda = 1, 잔차 미분 포함)'}, 'Location','northwest');
title(sprintf(['잔차 미분을 넣으면 MPC가 인식하는 조향 권한이 요동친다  ' ...
               '(%s)\n비율 중앙값 %.0f%%,  5~95%% 구간 %.0f~%.0f%%'], ...
       strrep(run,'_','\_'), 100*median(a1./a0), ...
       100*quantile(a1./a0,0.05), 100*quantile(a1./a0,0.95)));

% ---- (b) 스텝 간 요동 --------------------------------------------------
subplot(3,1,3); hold on; box on
plot(s, 100*j0, 'Color',[0.16 0.55 0.36], 'LineWidth',1.2);
plot(s, 100*j1, 'Color',[0.78 0.24 0.20], 'LineWidth',1.2);
xlim([min(s) max(s)]);
xlabel('주행 거리  s  [m]'); ylabel('스텝 간 변화율 [%]');
legend({sprintf('nominal (중앙값 %.1f%%)', 100*median(j0,'omitnan')), ...
        sprintf('first-order (중앙값 %.1f%%)', 100*median(j1,'omitnan'))}, ...
       'Location','northwest');

fprintf('\n[%s]  n = %d 스텝\n', run, numel(s));
fprintf('  권한 중앙값 : nominal %.2f  /  first-order %.2f  (%.0f%%)\n', ...
        median(a0), median(a1), 100*median(a1./a0));
fprintf('  스텝간 요동 : nominal %.1f%%  /  first-order %.1f%%  (%.1f배)\n\n', ...
        100*median(j0,'omitnan'), 100*median(j1,'omitnan'), ...
        median(j1,'omitnan')/median(j0,'omitnan'));
end

% ------------------------------------------------------------------------
function shade_corners(s, mask, yl)
d = diff([false; mask(:); false]);
for i = find(d == 1)'
    j = find(d(i+1:end) == -1, 1) + i;
    patch([s(i) s(j-1) s(j-1) s(i)], [yl(1) yl(1) yl(2) yl(2)], ...
          [0.88 0.88 0.88], 'EdgeColor','none', 'HandleVisibility','off');
end
patch(nan, nan, [0.88 0.88 0.88], 'EdgeColor','none');   % 범례용 더미
end
