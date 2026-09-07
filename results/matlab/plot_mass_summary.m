% 적재별 종합 - 예측 오차와 주행 오차를 한 자리에 (32 / 40 / 48 / 56 t)
%
% 무게를 MLP 입력에 넣을지에 대한 근거를 적재별로 모았다. 두 축이 있다.
%
%   가로축 = 무엇을 재나
%     예측 오차   1스텝 잔차 예측 오차. 모델 자체의 정확도
%     주행 오차   급코너 횡오차 RMS / 헤딩오차 RMS. 폐루프 추종 성능
%
%   세로축 = 그 적재를 학습에서 봤나
%     본 적재     32/40/48/56 을 다 섞어 학습. 평가 적재도 학습에 들어있다
%     안 본 적재  그 적재를 통째로 빼고 학습. 처음 보는 적재다
%
% '본 적재' 는 무게에 가장 유리한 조건이다 - 학습에서 봤고, 입력에 무게 칸이 있고,
% 주행 때 정답 무게까지 받는다. 실차에는 적재 중량계가 없으니 이건 상한선이다.
% '안 본 적재' 가 실제 배치 상황에 가깝고, 거기서 결론이 뒤집힌다.
%
%   본 적재    예측은 차이 없고(0~3%), 주행은 조건화가 7~20% 좋아 보인다.
%              그런데 플라시보(무게 열을 섞어 정보량 0)가 그 이득의 절반 이상을
%              재현한다 - 무게 정보가 아니라 입력 차원 탓이다.
%   안 본 적재 조건화가 **진다**. 횡 -24.8 / +4.0 / -54.4 / -112.5 %.
%              12 시드쌍 중 10쌍이 무게 X 승. 플라시보는 12/12 로 조건화를 이긴다.
%
% ** 40 t 주행 주의 ** 40 t 만 주행 조건이 다르다 (v=12 / 10 Hz / 시드 1개, 나머지는
% v=10 / 6.5 Hz / 시드 3개). 속도가 1.9 m/s 빨라 절대 오차가 약 2배다. 막대에
% 빗금으로 표시했다. **40 t 내부의 무게 O/X 쌍비교만 유효하다.**
%
% ** 안 본 적재 주행 ** 아직 주행 안 했으면 그 칸은 비고 '(not run yet)' 이 찍힌다.
% 채우려면:  ./scripts/run_ablation.sh lomo_nomass 56 0   (TruckMaker Loads=24000)
%
% 창 3개가 뜬다.
%   Figure 1  횡방향  4칸 (예측/본, 예측/안본, 주행/본, 주행/안본)
%   Figure 2  헤딩    4칸 (같은 배치)
%   Figure 3  같은 값을 표로. 명령창에도 같이 출력된다.
%
% 데이터: fig_mass_summary.mat  (results/mass_summary.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_mass_summary`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_mass_summary.mat')

GREY = [0.62 0.62 0.62];    % 보정 없음
NOM  = [0.12 0.47 0.71];    % 무게 없음
RES  = [0.90 0.49 0.13];    % 무게 있음
PLA  = [0.00 0.62 0.45];    % 플라시보 (Okabe-Ito bluish green)
COL  = [NOM; RES; PLA];

mm = mass(:);  nM = numel(mm);  x = (1:nM)';
w  = 0.24;
DN = 2; DA = 3;             % 예측 채널: 1=ds 2=dn 3=dalpha 4=dv

% NaN 안전 평균/표준편차 (플라시보는 일부 적재만, 40t 주행은 시드 1개, 안 본 적재
% 주행은 아직 없을 수 있다). nanmean 은 Statistics Toolbox 함수라 쓰지 않는다.
nm_mean = @(v) sum(v(~isnan(v))) / max(sum(~isnan(v)), 1);
nm_sd   = @(v) std(v(~isnan(v)));
nm_n    = @(v) sum(~isnan(v));

%% ---- 4칸의 정의: {데이터, 보정없음, 채널, 배율, 제목, y라벨} ----
% k=1 횡, k=2 헤딩
for k = 1:2
    if k == 1
        pc = DN; dc = 1; sc = 1;      chn = 'lateral';
        yp = '1-step  \Deltan  [m]';  yd = 'corner  |n|  RMS  [m]';
    else
        pc = DA; dc = 2; sc = 180/pi; chn = 'heading';
        yp = '1-step  \Delta\alpha  [deg]'; yd = 'corner  \alpha  RMS  [deg]';
    end
    PANEL = { pred_seen,   pred_raw_seen,   pc, sc, ...
                sprintf('(a)  prediction error  -  SEEN payload'),   yp, 0; ...
              pred_unseen, pred_raw_unseen, pc, sc, ...
                sprintf('(b)  prediction error  -  UNSEEN payload'), yp, 0; ...
              drive_seen,  [],              dc, 1,  ...
                sprintf('(c)  tracking error  -  SEEN payload'),     yd, 1; ...
              drive_unseen,[],              dc, 1,  ...
                sprintf('(d)  tracking error  -  UNSEEN payload'),   yd, 1 };

    figure('Color','w','Position',[40+30*k 40 1120 720])
    for p = 1:4
        A = PANEL{p,1};  R = PANEL{p,2};  ci = PANEL{p,3};
        s = PANEL{p,4};  ttl = PANEL{p,5}; ylb = PANEL{p,6}; isdrive = PANEL{p,7};
        subplot(2,2,p); hold on; box off
        set(gca,'FontSize',9,'TickDir','out','YGrid','on', ...
                'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')

        if ~isempty(R)      % 보정 없음: 회색 배경 막대 (스케일 기준선)
            bar(x, R(:,ci)*s, 0.72, 'FaceColor', GREY, 'EdgeColor','none', ...
                'FaceAlpha', 0.30);
        end

        hb = []; lg = {}; any_data = 0;
        for a = 1:3
            mu = zeros(nM,1); n = zeros(nM,1);
            for i = 1:nM
                v = squeeze(A(i,a,:,ci))*s;
                mu(i) = nm_mean(v);  n(i) = nm_n(v);
            end
            if ~any(n), continue; end
            any_data = 1;
            mu(n == 0) = 0;
            hh = bar(x + (a-2)*w, mu, w*0.9, 'FaceColor', COL(a,:), 'EdgeColor','none');
            hb(end+1) = hh; lg{end+1} = arm{a};  %#ok<AGROW,SAGROW>
            for i = 1:nM
                v = squeeze(A(i,a,:,ci))*s;  v = v(~isnan(v));
                if isempty(v), continue; end
                % 개별 시드를 점으로 - 산포를 숨기지 않는다
                plot(x(i) + (a-2)*w, v, 'o', 'MarkerSize', 3.5, ...
                     'MarkerFaceColor','w','MarkerEdgeColor',COL(a,:)*0.7,'LineWidth',0.8);
                % 40t 주행은 조건이 다르다 - 별표로 경고
                if isdrive && ~drive_same(i)
                    yl = ylim;
                    text(x(i)+(a-2)*w, mu(i)+0.03*(yl(2)-yl(1)), '*', ...
                         'HorizontalAlignment','center','FontSize',11,'Color',[.7 .2 .2]);
                end
            end
        end

        set(gca,'XTick',x,'XTickLabel',arrayfun(@(m) sprintf('%dt',m), mm, ...
                'UniformOutput',false))
        ylabel(ylb);  xlim([0.5 nM+0.5])
        title(ttl,'FontSize',10,'FontWeight','normal')
        if p >= 3, xlabel('Payload'); end
        if ~any_data
            yl = ylim;
            text(mean(xlim), mean(yl), '(not run yet)', ...
                 'HorizontalAlignment','center','FontSize',12,'Color',[.6 .6 .6]);
        elseif p == 1
            legend(hb, lg, 'Location','northwest','Box','off','FontSize',8);
        end
        if isdrive && any(~drive_same)
            yl = ylim;
            text(0.55, yl(1)+0.96*(yl(2)-yl(1)), ...
                 '*  different run condition (v=12, 10 Hz, 1 seed)', ...
                 'FontSize',7.5,'Color',[.7 .2 .2]);
        end
    end
    annotation('textbox',[0.005 0.955 0.99 0.04],'String', ...
        sprintf(['%s channel   -   with-mass input helps only where the payload ' ...
        'was in the training set'], upper(chn)), ...
        'EdgeColor','none','FontSize',11,'FontWeight','bold', ...
        'HorizontalAlignment','center');
end

%% ---- 수치 표 (명령창 + Figure 3) ----
LBL = {'lateral', 'heading'};
BLK = {'prediction / SEEN', 'prediction / UNSEEN', ...
       'tracking / SEEN',   'tracking / UNSEEN'};
lines = {};
for k = 1:2
    if k == 1, pc = DN; dc = 1; sc = 1; else pc = DA; dc = 2; sc = 180/pi; end
    SRC = {pred_seen, pred_raw_seen, pc, sc; pred_unseen, pred_raw_unseen, pc, sc; ...
           drive_seen, [], dc, 1;            drive_unseen, [], dc, 1};
    lines{end+1} = sprintf('=== %s ===', upper(LBL{k}));  %#ok<SAGROW>
    for b = 1:4
        A = SRC{b,1}; R = SRC{b,2}; ci = SRC{b,3}; s = SRC{b,4};
        lines{end+1} = sprintf('--- %s', BLK{b});  %#ok<SAGROW>
        lines{end+1} = sprintf('%6s %10s %16s %16s %16s %8s', ...
            'mass','no resid','no mass','with mass','shuffled','gain');  %#ok<SAGROW>
        for i = 1:nM
            if isempty(R), r0 = sprintf('%10s','-');
            else           r0 = sprintf('%10.4f', R(i,ci)*s); end
            c = cell(1,3); mu = nan(1,3);
            for a = 1:3
                v = squeeze(A(i,a,:,ci))*s;
                if nm_n(v) == 0, c{a} = sprintf('%16s','-');
                else
                    mu(a) = nm_mean(v);
                    c{a} = sprintf('%10.4f%c%.4f', mu(a), char(177), nm_sd(v));
                end
            end
            if isnan(mu(1)) || isnan(mu(2)), g = sprintf('%8s','-');
            else g = sprintf('%7.1f%%', 100*(mu(1)-mu(2))/mu(1)); end
            lines{end+1} = sprintf('%5dt %s %s %s %s %s', ...
                mm(i), r0, c{1}, c{2}, c{3}, g);  %#ok<SAGROW>
        end
    end
    lines{end+1} = '';  %#ok<SAGROW>
end
fprintf('\n');
for i = 1:numel(lines), fprintf('%s\n', lines{i}); end

figure('Color','w','Position',[100 60 880 900]); axis off
text(0.02, 0.985, 'Payload summary — prediction error and tracking error', ...
     'FontSize',12,'FontWeight','bold');
yy = 0.955;
for i = 1:numel(lines)
    L = lines{i};
    if isempty(L), yy = yy - 0.010; continue; end
    if strncmp(L,'===',3),      fw='bold'; cl=[0 0 0];        fs=10;
    elseif strncmp(L,'---',3),  fw='bold'; cl=[.25 .25 .25];  fs=9;
    elseif strncmp(L,'  mass',6) || ~isempty(strfind(L,'no resid'))
                                fw='bold'; cl=[.4 .4 .4];     fs=8.5;
    else                        fw='normal'; cl=[0 0 0];      fs=8.5; end
    text(0.02, yy, L, 'FontName','FixedWidth','FontSize',fs, ...
         'FontWeight',fw,'Color',cl,'Interpreter','none');
    yy = yy - 0.0215;
end
xlim([0 1]); ylim([0 1])
