% 적재 조건화 근거 - MLP 입력에 무게를 넣어야 하는가
%
% (a)(b) 예측 오차: 잔차 보정은 크게 줄이지만, 무게를 알려줘도 거의 안 줄어든다.
%        Δn 기준 조건화 이득이 32t 3.0%, 40t 2.5%, 48t 0.1%, 56t 0.1% 다.
% (c)    폐루프에서는 조건화가 11~20% 좋아 보인다. 그러나 무게 라벨을 무작위로
%        섞은 플라시보(48 t)가 그 이득의 51%(횡) / 112%(헤딩)를 재현한다.
%        즉 폐루프 차이는 '무게 정보'가 아니라 입력 차원/초기값/학습 노이즈다.
%
% 결론: 무게를 입력에 넣을 근거가 없다. 무게는 이미 상태(D, v 등)에 인코딩돼
%       있다(results/payload_observability.py 참조).
%
% 데이터: fig_mass_conditioning.mat  (results/mass_conditioning.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_mass_conditioning`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_mass_conditioning.mat')

GREY = [0.62 0.62 0.62];       % 보정 없음
NOM  = [0.12 0.47 0.71];       % 무게 없음
RES  = [0.90 0.49 0.13];       % 무게 있음
PLA  = [0.00 0.62 0.45];       % 플라시보 (Okabe-Ito bluish green)

mp = mass_pred(:);  nM = numel(mp);
x  = (1:nM)';  w = 0.24;
DN = 2; DA = 3;                % 채널 인덱스: 1=ds 2=dn 3=dalpha 4=dv

figure('Color','w','Position',[60 60 1280 380])

%% (a),(b) 예측 오차 - 무게를 알려줘도 안 줄어든다
for k = 1:2
    if k == 1
        ch = DN;  sc = 1;          ttl = '(a)  Lateral prediction error';
        yl = '1-step \Deltan error  [m]';
    else
        ch = DA;  sc = 180/pi;     ttl = '(b)  Heading prediction error';
        yl = '1-step \Delta\alpha error  [deg]';
    end
    subplot(1,3,k); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
            'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
    e0 = squeeze(pred_err(:,1,1,ch))*sc;                 % 보정 없음
    e1 = mean(squeeze(pred_err(:,2,:,ch)),2)*sc;         % 무게 없음
    e2 = mean(squeeze(pred_err(:,3,:,ch)),2)*sc;         % 무게 있음
    b0 = bar(x-w, e0, w, 'FaceColor', GREY, 'EdgeColor','none');
    b1 = bar(x,   e1, w, 'FaceColor', NOM,  'EdgeColor','none');
    b2 = bar(x+w, e2, w, 'FaceColor', RES,  'EdgeColor','none');
    for i = 1:nM
        g = 100*(e1(i)-e2(i))/e1(i);
        text(x(i)+w, e2(i)+max(e0)*0.05, sprintf('%+.0f%%', g), ...
             'HorizontalAlignment','center','FontSize',9,'Color',RES);
    end
    if k == 1
        legend([b0 b1 b2], {'no residual','residual, no mass','residual, with mass'}, ...
               'Location','northwest','Box','off','FontSize',8.5);
    end
    set(gca,'XTick',1:nM,'XTickLabel', ...
            arrayfun(@(v) sprintf('%dt',v), mp, 'UniformOutput', false))
    xlim([0.4 nM+0.6]); ylim([0 max(e0)*1.28])
    xlabel('Payload'); ylabel(yl); title(ttl,'FontSize',11,'FontWeight','normal')
end

%% (c) 폐루프 + 플라시보
subplot(1,3,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
mc = mass_cloop(:); nC = numel(mc); xc = (1:nC)';
c0 = nanmean(squeeze(cloop_err(:,1,:,1)),2);       % 무게 없음
c1 = nanmean(squeeze(cloop_err(:,2,:,1)),2);       % 무게 있음
c2 = nanmean(squeeze(cloop_err(:,3,:,1)),2);       % 플라시보 (48t 만)
h1 = bar(xc-w, c0, w, 'FaceColor', NOM, 'EdgeColor','none');
h2 = bar(xc,   c1, w, 'FaceColor', RES, 'EdgeColor','none');
h3 = bar(xc+w, c2, w, 'FaceColor', PLA, 'EdgeColor','none');
% 시드별 개별 점 - 평균만 보면 3회 편차가 안 보인다
off = [-w 0 w];
for v = 1:3
    for i = 1:nC
        s = squeeze(cloop_err(i,v,:,1)); s = s(~isnan(s));
        plot(xc(i)+off(v)+zeros(size(s)), s, 'o', 'Color','k', ...
             'MarkerSize',3.5,'LineWidth',0.8);
    end
end
i48 = find(mc == 48);
if ~isempty(i48)
    text(xc(i48)+w, c2(i48)+0.045, 'placebo', 'HorizontalAlignment','center', ...
         'FontSize',9,'Color',PLA);
end
legend([h1 h2 h3], {'no mass','with mass','shuffled mass'}, ...
       'Location','northwest','Box','off','FontSize',8.5);
set(gca,'XTick',1:nC,'XTickLabel', ...
        arrayfun(@(v) sprintf('%dt',v), mc, 'UniformOutput', false))
xlim([0.4 nC+0.6]); ylim([0 0.56])
xlabel('Payload'); ylabel('Closed-loop corner |n| RMS  [m]')
title('(c)  Placebo reproduces the gain','FontSize',11,'FontWeight','normal')
