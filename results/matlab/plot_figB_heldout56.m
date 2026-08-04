%PLOT_FIGB_HELDOUT56  [주장 B·C] 미지 적재(56 t)에서의 성능.
%   56 t 를 학습·val·정규화 통계에서 완전히 제외한 모델로 56 t 주행 (시드 3개).
%   B: 무게-blind 가 미지 적재에서도 nominal 대비 크게 개선되는가  -> 그렇다
%   C: 무게를 넣은 모델이 미지 적재에서 더 나은가                  -> 아니다

clear; close all;
SEED = 0:2;
KTHR = 0.020;                     % 급코너 R <= 50 m
T    = load_track();

nom  = run_stats('nom_56', T, KTHR);
arms = {'heldout56_blind', 'State-only'; 'heldout56_cond', 'Mass-conditioned'};
V = nan(2, numel(SEED), 2);       % (arm, seed, [mean corner])
for a = 1:2
    for j = 1:numel(SEED)
        q = run_stats(sprintf('%s_s%d', arms{a,1}, SEED(j)), T, KTHR);
        if isempty(q), continue; end
        V(a,j,1) = q.mean;  V(a,j,2) = q.corner;
    end
end
inb = nan(numel(SEED),2);         % 참고: 56t 를 학습에 포함한 blind
for j = 1:numel(SEED)
    q = run_stats(sprintf('blind_56_s%d', SEED(j)), T, KTHR);
    if ~isempty(q), inb(j,:) = [q.mean q.corner]; end
end

figure('Color','w','Position',[80 80 1000 420]);
ttl = {'Mean |n| [m]', sprintf('Corner RMS |n| [m]  (R \\leq %.0f m)', 1/KTHR)};
ref = [nom.mean nom.corner];
for p = 1:2
    subplot(1,2,p); hold on; grid on; box on;
    mu = [mean(V(1,:,p)) mean(V(2,:,p))];
    bar(mu);
    for a = 1:2
        plot(a + zeros(1,numel(SEED)), squeeze(V(a,:,p)), 'ko', 'MarkerFaceColor','w');
        errorbar(a, mu(a), std(squeeze(V(a,:,p))), 'k', 'LineStyle','none','LineWidth',1.2);
    end
    ylim([0 1.15*max([ref(p) mu])]);
    hline(ref(p), [.4 .4 .4], sprintf('nominal %.3f', ref(p)));
    hline(mean(inb(:,p)), [0 .6 .3], sprintf('trained-on-56t %.3f', mean(inb(:,p))));
    set(gca,'XTick',1:2,'XTickLabel',arms(:,2));
    ylabel(ttl{p});
    if p==1, title('Unseen payload (56 t held out)'); else, title('Sharp corners'); end
end

fprintf('[B/C] 미지 적재 56t (nominal mean %.3f, corner %.3f)\n', nom.mean, nom.corner);
for a = 1:2
    fprintf('  %-18s mean %.3f±%.3f (%+.0f%% vs nom)   corner %.3f±%.3f (%+.0f%%)\n', arms{a,2}, ...
        mean(V(a,:,1)), std(V(a,:,1)), 100*(mean(V(a,:,1))/nom.mean-1), ...
        mean(V(a,:,2)), std(V(a,:,2)), 100*(mean(V(a,:,2))/nom.corner-1));
end
fprintf('  차이(cond - blind): mean %+.3f   corner %+.3f\n', ...
        mean(V(2,:,1))-mean(V(1,:,1)), mean(V(2,:,2))-mean(V(1,:,2)));
