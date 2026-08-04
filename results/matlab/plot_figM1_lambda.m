%PLOT_FIGM1_LAMBDA  잔차 Jacobian 주입(lambda) 스윕 — 값만(lambda=0)일 때만 완주.
%   figM1_lambda_sweep.png / figT7_authority_collapse.png 재현.
%   데이터: dbglog/*.mat  (12열: s,n,alpha,v,D,delta,bjac_norm,res_*,tgt_delta,status)

clear; close all;
runs = {'ff',      '\lambda = 0 (값만, FF)',   [0 .45 .74]; ...
        'js025',   '\lambda = 0.25',           [.93 .69 .13]; ...
        'js05',    '\lambda = 0.5',            [.85 .33 .10]; ...
        'js10',    '\lambda = 1.0',            [.64 .08 .18]; ...
        'jsneg05', '\lambda = -0.5',           [.49 .18 .56]};
R2D = 180/pi;  LAP = 2540;

figure('Color', 'w', 'Position', [100 100 1000 700]);
fprintf('%-9s %6s %8s %9s %11s %9s\n', 'run', 'N', 's_max', '|n|max', 'd|delta|', 'status~=0');
for i = 1:size(runs,1)
    a  = load(sprintf('dbglog/%s.mat', runs{i,1}));
    dd = abs(diff(a.delta)) * R2D;                 % 스텝당 조향 변화 [deg]

    subplot(2,1,1); hold on;
    plot(a.s, abs(a.n), 'LineWidth', 1.4, 'Color', runs{i,3});

    subplot(2,1,2); hold on;
    plot(a.s(2:end), dd, 'LineWidth', 1.0, 'Color', runs{i,3});

    fprintf('%-9s %6d %8.1f %9.3f %11.3f %9d\n', runs{i,1}, numel(a.s), ...
            max(a.s), max(abs(a.n)), mean(dd), sum(a.status ~= 0));
end

subplot(2,1,1); grid on; box on;
plot([0 LAP], [1 1], 'k--');                       % 소프트 제약 |n| <= 1 m
xlabel('s [m]'); ylabel('|n| [m]'); set(gca, 'YScale', 'log');
title('잔차 Jacobian 주입: \lambda \neq 0 은 부호·크기 무관하게 이탈');
legend([runs(:,2); {'|n| 제약 1 m'}], 'Location', 'southeast');

subplot(2,1,2); grid on; box on;
xlabel('s [m]'); ylabel('|\Delta\delta| [deg/step]');
title('조향 활동도 (발산의 서명: FF 대비 4~5배)');
