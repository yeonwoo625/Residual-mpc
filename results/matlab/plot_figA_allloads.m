%PLOT_FIGA_ALLLOADS  [주장 A] 무게 입력 없는 단일 잔차 모델이 전 적재에서 작동.
%   32/48/56 t 에서 nominal MPC 대비 횡오차 감소 (시드 3개).
%   점 = 개별 시드, 막대 = 평균, 오차막대 = 표준편차.

clear; close all;
M    = [32 48 56];
SEED = 0:2;
KTHR = 0.020;                 % 급코너 R <= 50 m
T    = load_track();

nomM = zeros(numel(M),1);  nomC = zeros(numel(M),1);
bM = nan(numel(M), numel(SEED));  bC = nan(numel(M), numel(SEED));
for i = 1:numel(M)
    q = run_stats(sprintf('nom_%d', M(i)), T, KTHR);
    nomM(i) = q.mean;  nomC(i) = q.corner;
    for j = 1:numel(SEED)
        q = run_stats(sprintf('blind_%d_s%d', M(i), SEED(j)), T, KTHR);
        if isempty(q), continue; end
        bM(i,j) = q.mean;  bC(i,j) = q.corner;
    end
end

figure('Color','w','Position',[80 80 1000 420]);
data = {nomM, bM, 'Mean |n| [m]'; nomC, bC, sprintf('Corner RMS |n| [m]  (R \\leq %.0f m)', 1/KTHR)};
for p = 1:2
    subplot(1,2,p); hold on; grid on; box on;
    nomv = data{p,1};  bv = data{p,2};
    h = bar([nomv mean(bv,2)]);
    for i = 1:numel(M)
        plot(i + 0.15 + zeros(1,numel(SEED)), bv(i,:), 'ko', 'MarkerFaceColor','w');
        errorbar(i + 0.15, mean(bv(i,:)), std(bv(i,:)), 'k', 'LineStyle','none','LineWidth',1.2);
    end
    set(gca,'XTick',1:numel(M),'XTickLabel',{'32 t','48 t','56 t'});
    ylabel(data{p,3});
    if p == 1
        legend(h, {'Nominal MPC','Residual MPC (no mass input)'}, 'Location','northwest');
        title('Single mass-blind residual across all payloads');
    else
        title('Sharp corners');
    end
end

fprintf('[A] 전 적재 성능 (1랩+거리가중, 급코너 R<=%.0f m)\n', 1/KTHR);
fprintf('%5s | %8s %18s %7s | %8s %18s %7s\n','적재','nom','blind mean','개선','nom','blind corner','개선');
for i = 1:numel(M)
    fprintf('%4dt | %8.3f %11.3f±%.3f %6.0f%% | %8.3f %11.3f±%.3f %6.0f%%\n', M(i), ...
        nomM(i), mean(bM(i,:)), std(bM(i,:)), 100*(mean(bM(i,:))/nomM(i)-1), ...
        nomC(i), mean(bC(i,:)), std(bC(i,:)), 100*(mean(bC(i,:))/nomC(i)-1));
end
