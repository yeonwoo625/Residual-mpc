% 조향 각속도 제약 민감도 - 성능 이득이 완화된 제약 덕분인가?
%
% 기본 제약 57.3 deg/s (앞바퀴) 는 시뮬레이터 기준 운전자가 같은 코스에서 쓴
% 최대값(2.47 deg/s)의 23배다. 3.8배 강화(15 deg/s)해도 결론이 유지되는지 확인.
%
%   개선율   58% -> 54% (v=10),  17% -> 19% (v=12)   = 유지
%   조향채터 39~48% 감소                              = 오히려 개선
%
% 데이터: fig_ddelta.mat  (results/ddelta_summary.py 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_ddelta`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

load('fig_ddelta.mat')

NOM  = [0.12 0.47 0.71];      % 파랑 - nominal
RES  = [0.90 0.49 0.13];      % 주황 - residual
GRY  = [0.45 0.45 0.45];

figure('Color','w','Position',[80 80 980 380])

%% (a) 제약별 개선율 - 유지되는가
subplot(1,2,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
sp = [10 12];
for i = 1:2
    a = improve(v_target==sp(i) & limit==limit_loose);
    b = improve(v_target==sp(i) & limit==limit_tight);
    x = (i-1)*1.6;
    bar(x+0.7, a, 0.32, 'FaceColor', GRY, 'EdgeColor','none');
    bar(x+1.05, b, 0.32, 'FaceColor', RES, 'EdgeColor','none');
    text(x+0.7,  a+1.5, sprintf('%.1f%%', a), 'HorizontalAlignment','center','FontSize',9);
    text(x+1.05, b+1.5, sprintf('%.1f%%', b), 'HorizontalAlignment','center','FontSize',9);
end
set(gca,'XTick',[0.875 2.475],'XTickLabel',{'v = 10 m/s','v = 12 m/s'})
ylabel('Improvement of residual over nominal  [%]')
ylim([0 max(improve)*1.3]); xlim([0.3 3.0])
h(1)=patch(nan,nan,GRY,'EdgeColor','none'); h(2)=patch(nan,nan,RES,'EdgeColor','none');
legend(h, {sprintf('%.1f deg/s (default)', limit_loose), ...
           sprintf('%.0f deg/s (tightened)', limit_tight)}, ...
       'Location','northeast','Box','off','FontSize',9)

%% (b) 제약별 조향 채터 - 오히려 개선
subplot(1,2,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
lbl = {}; xi = 0;
for i = 1:2
    for w = 0:1     % 0 = nominal, 1 = residual
        xi = xi + 1;
        if w == 0
            a = chat_nom(v_target==sp(i) & limit==limit_loose);
            b = chat_nom(v_target==sp(i) & limit==limit_tight);
            c = NOM;
        else
            a = chat_res(v_target==sp(i) & limit==limit_loose);
            b = chat_res(v_target==sp(i) & limit==limit_tight);
            c = RES;
        end
        bar(xi-0.17, a, 0.3, 'FaceColor', c, 'FaceAlpha', 0.35, 'EdgeColor','none');
        bar(xi+0.17, b, 0.3, 'FaceColor', c, 'EdgeColor','none');
        text(xi, max(a,b)*1.06, sprintf('%+.0f%%', 100*(b/a-1)), ...
             'HorizontalAlignment','center','FontSize',9,'Color',GRY);
        lbl{xi} = sprintf('v%d %s', sp(i), char('nom'*(w==0) + 'res'*(w==1)));
    end
end
set(gca,'XTick',1:4,'XTickLabel',lbl)
ylabel('Steering chatter  RMS(d\delta/dt)')
xlim([0.4 4.6]); ylim([0 max([chat_nom;chat_res])*1.25])
h2(1)=patch(nan,nan,[.5 .5 .5],'FaceAlpha',0.35,'EdgeColor','none');
h2(2)=patch(nan,nan,[.5 .5 .5],'EdgeColor','none');
legend(h2, {sprintf('%.1f deg/s', limit_loose), sprintf('%.0f deg/s', limit_tight)}, ...
       'Location','northwest','Box','off','FontSize',9)

%% 콘솔 요약
fprintf('\n[조향 각속도 제약 민감도]  기준 운전자 최대 %.2f deg/s\n', ipg_driver_max);
for i = 1:2
    a = improve(v_target==sp(i) & limit==limit_loose);
    b = improve(v_target==sp(i) & limit==limit_tight);
    ca = chat_res(v_target==sp(i) & limit==limit_loose);
    cb = chat_res(v_target==sp(i) & limit==limit_tight);
    fprintf('  v=%2d : 개선율 %.1f%% -> %.1f%%   채터(res) %.4f -> %.4f (%+.0f%%)\n', ...
            sp(i), a, b, ca, cb, 100*(cb/ca-1));
end
fprintf('\n  -> 제약을 %.1f배 강화해도 개선율 유지, 조향 채터는 감소\n\n', ...
        limit_loose/limit_tight);
