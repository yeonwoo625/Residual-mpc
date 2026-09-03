% 적재 조건화 - MLP 입력에 무게를 넣어야 하는가 (32 / 40 / 48 / 56 t)
%
% (a)(b) 예측 오차 - 모델이 실제로 하는 일. 학습에 쓰지 않은 검증 분할에서
%        1스텝 잔차 예측 오차를 잰다. 잔차 보정은 오차를 8배 줄이지만,
%        무게를 알려줘도 Δn 은 3.0 / 2.5 / 0.1 / 0.1 % 밖에 안 줄고
%        Δα 는 +7.4 / +4.2 / -7.0 / -3.3 % 로 부호까지 갈린다.
% (c)(d) 폐루프 추종 오차 - 실주행. 조건화가 7~20% 좋아 보이지만, 무게 라벨을
%        무작위로 섞은 플라시보(48 t)가 그 이득의 51%(횡) / 112%(헤딩)를
%        재현한다. 즉 무게 정보가 아니라 입력 차원/초기값/학습 노이즈다.
%
% ** 40 t 주의 ** 40 t 폐루프만 주행 조건이 다르다 (v=12 / 10 Hz / 시드 1개,
% 나머지는 v=10 / 6.5 Hz / 시드 3개). 속도가 1.9 m/s 빨라 절대 오차가 약 2배다.
% 막대에 빗금으로 표시했다. **40 t 내부의 무게 O/X 쌍비교만 유효하다.**
%
% 데이터: fig_mass_conditioning.mat  (results/mass_conditioning.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_mass_conditioning`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_mass_conditioning.mat')

GREY = [0.62 0.62 0.62];    % 보정 없음
NOM  = [0.12 0.47 0.71];    % 무게 없음
RES  = [0.90 0.49 0.13];    % 무게 있음
PLA  = [0.00 0.62 0.45];    % 플라시보 (Okabe-Ito bluish green)

mp = mass_pred(:);   nP = numel(mp);   xp = (1:nP)';
mc = mass_cloop(:);  nC = numel(mc);   xc = (1:nC)';
w  = 0.24;
DN = 2; DA = 3;             % 채널: 1=ds 2=dn 3=dalpha 4=dv

% NaN 안전 평균 (플라시보는 48t 만, 40t 는 시드 1개라 빈 칸이 있다).
% nanmean 은 Statistics Toolbox 함수라 쓰지 않는다 - base MATLAB 만으로 계산한다.
Zs  = cloop_err;  Zs(isnan(Zs)) = 0;
SUM = squeeze(sum(Zs, 3));                      % (적재, 모델, 지표)
CNT = squeeze(sum(~isnan(cloop_err), 3));
CM  = SUM ./ max(CNT, 1);
CM(CNT == 0) = NaN;

%% ---- 수치 표 출력 (명령창) ----
% 그림만 보면 0.1% 대 차이가 안 보인다. 숫자로도 남긴다.
fprintf('\n');
fprintf('=== Mass conditioning: prediction error (held-out val, n=%d) ===\n', n_val);
fprintf('%5s | %9s %9s %9s %9s | %7s\n', 'mass', ...
        'no resid', 'no mass', 'with mass', 'shuffled', 'gain');
for k = 1:2
    if k == 1, ch = DN; sc = 1;      fprintf('--- lateral  dn [m]\n');
    else       ch = DA; sc = 180/pi; fprintf('--- heading  dalpha [deg]\n'); end
    for i = 1:nP
        e0 = pred_err(i,1,1,ch)*sc;
        e1 = mean(pred_err(i,2,:,ch))*sc;
        e2 = mean(pred_err(i,3,:,ch))*sc;
        e3 = mean(pred_err(i,4,:,ch))*sc;
        fprintf('%4dt | %9.4f %9.4f %9.4f %9.4f | %+6.1f%%\n', ...
                mp(i), e0, e1, e2, e3, 100*(e1-e2)/e1);
    end
end

fprintf('\n=== Mass conditioning: closed-loop tracking ===\n');
fprintf('%5s | %9s %9s %9s | %7s | %s\n', 'mass', ...
        'no mass', 'with mass', 'shuffled', 'gain', 'run condition');
for k = 1:2
    if k == 1, ci = 1; fprintf('--- corner |n| RMS [m]\n');
    else       ci = 2; fprintf('--- corner alpha RMS [deg]\n'); end
    for i = 1:nC
        g = 100*(CM(i,1,ci)-CM(i,2,ci))/CM(i,1,ci);
        note = ''; if k == 1, note = cloop_note{i}; end
        fprintf('%4dt | %9.3f %9.3f %9.3f | %+6.1f%% | %s\n', ...
                mc(i), CM(i,1,ci), CM(i,2,ci), CM(i,3,ci), g, note);
    end
end
fprintf(['\ngain = (no mass - with mass) / no mass.  ' ...
         'Positive = conditioning looks better.\n']);
fprintf(['Placebo (shuffled mass) carries no mass information but the same ' ...
         '8-dim input.\n']);
fprintf(['40t closed-loop ran at a different condition - compare within 40t ' ...
         'only.\n\n']);

figure('Color','w','Position',[50 40 1060 720])

%% (a)(b) 예측 오차 - 무게를 알려줘도 안 줄어든다
for k = 1:2
    if k == 1
        ch = DN; sc = 1;      ttl = '(a)  Prediction error - lateral';
        yl = '1-step \Deltan error  [m]';
    else
        ch = DA; sc = 180/pi; ttl = '(b)  Prediction error - heading';
        yl = '1-step \Delta\alpha error  [deg]';
    end
    subplot(2,2,k); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
            'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
    e0 = squeeze(pred_err(:,1,1,ch))*sc;
    e1 = mean(squeeze(pred_err(:,2,:,ch)),2)*sc;
    e2 = mean(squeeze(pred_err(:,3,:,ch)),2)*sc;
    b0 = bar(xp-w, e0, w, 'FaceColor', GREY, 'EdgeColor','none');
    b1 = bar(xp,   e1, w, 'FaceColor', NOM,  'EdgeColor','none');
    b2 = bar(xp+w, e2, w, 'FaceColor', RES,  'EdgeColor','none');
    for i = 1:nP
        text(xp(i)+w, e2(i)+max(e0)*0.05, ...
             sprintf('%+.0f%%', 100*(e1(i)-e2(i))/e1(i)), ...
             'HorizontalAlignment','center','FontSize',9,'Color',RES);
    end
    if k == 1
        legend([b0 b1 b2], {'no residual','residual, no mass','residual, with mass'}, ...
               'Location','northwest','Box','off','FontSize',8.5);
    end
    set(gca,'XTick',1:nP,'XTickLabel', ...
            arrayfun(@(v) sprintf('%dt',v), mp, 'UniformOutput', false))
    xlim([0.4 nP+0.6]); ylim([0 max(e0)*1.3])
    xlabel('Payload'); ylabel(yl); title(ttl,'FontSize',11,'FontWeight','normal')
end

%% (c)(d) 폐루프 추종 오차 + 플라시보
for k = 1:2
    if k == 1
        ci = 1; ttl = '(c)  Closed-loop lateral error';
        yl = 'Corner |n| RMS  [m]';
    else
        ci = 2; ttl = '(d)  Closed-loop heading error';
        yl = 'Corner \alpha RMS  [deg]';
    end
    subplot(2,2,2+k); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
            'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
    c0 = CM(:,1,ci);  c1 = CM(:,2,ci);  c2 = CM(:,3,ci);
    h1 = bar(xc-w, c0, w, 'FaceColor', NOM, 'EdgeColor','none');
    h2 = bar(xc,   c1, w, 'FaceColor', RES, 'EdgeColor','none');
    h3 = bar(xc+w, c2, w, 'FaceColor', PLA, 'EdgeColor','none');
    % 주행 조건이 다른 적재(40t)는 테두리로 표시 - 절대값 비교 불가
    for i = 1:nC
        if ~cloop_same(i)
            bar(xc(i)-w, c0(i), w, 'FaceColor','none', ...
                'EdgeColor',[0.2 0.2 0.2],'LineWidth',1.2,'LineStyle','--');
            bar(xc(i),   c1(i), w, 'FaceColor','none', ...
                'EdgeColor',[0.2 0.2 0.2],'LineWidth',1.2,'LineStyle','--');
            text(xc(i), max(c0(i),c1(i))*1.11, 'different condition', ...
                 'HorizontalAlignment','center','FontSize',8,'Color',[0.2 0.2 0.2]);
        end
    end
    % 시드별 개별 점 - 평균만 보면 편차가 안 보인다
    off = [-w 0 w];
    for v = 1:3
        for i = 1:nC
            sd = squeeze(cloop_err(i,v,:,ci)); sd = sd(~isnan(sd));
            plot(xc(i)+off(v)+zeros(size(sd)), sd, 'o', 'Color','k', ...
                 'MarkerSize',3.5,'LineWidth',0.8);
        end
    end
    if k == 1
        legend([h1 h2 h3], {'no mass','with mass','shuffled mass (placebo)'}, ...
               'Location','northwest','Box','off','FontSize',8.5);
    end
    set(gca,'XTick',1:nC,'XTickLabel', ...
            arrayfun(@(v) sprintf('%dt',v), mc, 'UniformOutput', false))
    xlim([0.4 nC+0.6]); ylim([0 max([c0;c1])*1.30])
    xlabel('Payload'); ylabel(yl); title(ttl,'FontSize',11,'FontWeight','normal')
end
