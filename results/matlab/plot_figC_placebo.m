%PLOT_FIGC_PLACEBO  [주장 D] 조건화의 이득은 '무게 정보'가 아니다.
%   48 t 에서 4-way 비교 (잔차 모델은 시드 3개씩):
%     nominal            잔차 없음
%     blind    (6dim)    무게 입력 없음
%     placebo  (8dim)    무게 열을 무작위로 섞어 학습 -> 정보량 0, 차원만 동일
%     cond     (8dim)    진짜 무게
%   placebo ≈ cond 이면, 이득은 무게 정보가 아니라 입력 차원/용량 효과다.

clear; close all;
SEED = 0:2;
KTHR = 0.020;
T    = load_track();

nom  = run_stats('nom_48', T, KTHR);
arms = {'blind_48',    'State-only (6-D)'; ...
        'shufmass_48', 'Shuffled mass (8-D, no info)'; ...
        'cond_48',     'True mass (8-D)'};
V = nan(3, numel(SEED), 2);
for a = 1:3
    for j = 1:numel(SEED)
        q = run_stats(sprintf('%s_s%d', arms{a,1}, SEED(j)), T, KTHR);
        if isempty(q), continue; end
        V(a,j,1) = q.mean;  V(a,j,2) = q.corner;
    end
end

figure('Color','w','Position',[80 80 1050 430]);
ttl = {'Mean |n| [m]', sprintf('Corner RMS |n| [m]  (R \\leq %.0f m)', 1/KTHR)};
ref = [nom.mean nom.corner];
for p = 1:2
    subplot(1,2,p); hold on; grid on; box on;
    mu = mean(V(:,:,p), 2);
    bar(mu);
    for a = 1:3
        plot(a + zeros(1,numel(SEED)), squeeze(V(a,:,p)), 'ko', 'MarkerFaceColor','w');
        errorbar(a, mu(a), std(squeeze(V(a,:,p))), 'k', 'LineStyle','none','LineWidth',1.2);
    end
    ylim([0 1.15*max([ref(p); mu])]);
    hline(ref(p), [.4 .4 .4], sprintf('nominal %.3f', ref(p)));
    set(gca,'XTick',1:3,'XTickLabel',arms(:,2),'XTickLabelRotation',12);
    ylabel(ttl{p});
    if p==1
        title('48 t: is the conditioning gain about mass information?');
    else
        title('Sharp corners');
    end
end

fprintf('[D] 플라시보 검증 @48t (nominal mean %.3f, corner %.3f)\n', nom.mean, nom.corner);
for a = 1:3
    fprintf('  %-30s mean %.3f±%.3f   corner %.3f±%.3f\n', arms{a,2}, ...
        mean(V(a,:,1)), std(V(a,:,1)), mean(V(a,:,2)), std(V(a,:,2)));
end
fprintf('  placebo - cond : mean %+.3f   corner %+.3f   <- 0 에 가까우면 무게 정보 무관\n', ...
        mean(V(2,:,1))-mean(V(3,:,1)), mean(V(2,:,2))-mean(V(3,:,2)));
fprintf('  placebo - blind: mean %+.3f   corner %+.3f\n', ...
        mean(V(2,:,1))-mean(V(1,:,1)), mean(V(2,:,2))-mean(V(1,:,2)));
