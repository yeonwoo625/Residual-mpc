% 안 본 적재로 일반화되는가 - leave-one-mass-out
%
% 기존 조건화 ablation(plot_mass_conditioning.m)은 32/40/48/56 t 를 **전부 섞어**
% 학습한 뒤 그중 20%로 검증했다. 주행하는 적재를 학습에서 이미 본 셈이라
% "안 본 적재에서도 되는가" 에는 답하지 못한다.
%
% 여기서는 적재 하나를 통째로 빼고 학습한 뒤, 그 적재에서만 평가한다.
%   학습 풀  나머지 세 적재 (이 안에서 다시 80/20, val 은 best epoch 선택용)
%   시험     빼놓은 적재 전체. 학습에도 val 에도 한 번도 안 들어갔다.
%
% 세 팔을 같은 시드로 학습한다. 차이는 입력의 무게 열뿐이다.
%   무게 X    6차원. 무게를 안 준다
%   무게 O    8차원. 시험 때 **학습에 없던 무게값**을 받는다
%   섞은무게  8차원 플라시보. 차원은 같고 정보만 없다
%
% 결과: 안 본 적재에서는 **무게를 넣은 쪽이 진다.**
%   횡 dn    조건화 이득 -24.8 / +4.0 / -54.4 / -112.5 %  (12쌍 중 10쌍이 무게 X 승)
%   헤딩 da  조건화 이득 -54.7 / +2.8 / -22.0 / -108.9 %  (12쌍 중 9쌍이 무게 X 승)
%
% 기전은 플라시보가 말해준다. **섞은무게가 무게 O 를 12/12(횡) 이긴다.** 그리고
% 섞은무게는 무게 X 와 거의 같다(횡 차이 +0.00023 m). 무게 열이 학습 때 잡음이면
% 신경망은 그 열을 무시하는 법을 배우고, 무시하니까 안 본 적재에서도 멀쩡하다.
% 반대로 진짜 무게를 주면 신경망은 본 네 값에 맞춘 사상을 배우고, 다섯 번째 값이
% 들어오면 그 사상이 깨진다. 시드 산포도 무게 O 가 3.9배(횡) / 6.6배(헤딩) 크다.
%
% 40 t 만 조건화가 근소하게 이긴다(+4.0 / +2.8%). 40 t 는 학습 범위 32~56 t 의
% 안쪽이라 내삽이다. 범위 끝(32 / 56 t)에서 조건화가 가장 크게 무너진다.
%
% 창 2개가 뜬다.
%   Figure 1  그림 2칸 (횡 dn / 헤딩 dalpha). 막대 = 3시드 평균, 점 = 개별 시드
%   Figure 2  같은 값을 표로. 명령창에도 같이 출력된다.
%
% 데이터: fig_lomo_mass.mat  (results/lomo_mass.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_lomo_mass`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_lomo_mass.mat')

GREY = [0.62 0.62 0.62];    % 보정 없음
NOM  = [0.12 0.47 0.71];    % 무게 없음
RES  = [0.90 0.49 0.13];    % 무게 있음
PLA  = [0.00 0.62 0.45];    % 플라시보 (Okabe-Ito bluish green)
COL  = [NOM; RES; PLA];

mm = mass(:);  nM = numel(mm);  x = (1:nM)';
w  = 0.24;
DN = 2; DA = 3;             % 채널: 1=ds 2=dn 3=dalpha 4=dv
CH  = [DN DA];
SC  = [1 180/pi];
LBL = {'Lateral  \Deltan  [m]', 'Heading  \Delta\alpha  [deg]'};
TIT = {'(a)  lateral residual on the held-out payload', ...
       '(b)  heading residual on the held-out payload'};

%% ---- 수치 표 출력 (명령창) ----
fprintf('\n=== Leave-one-mass-out: prediction error on the UNSEEN payload ===\n');
fprintf('(model never saw this payload in training or validation)\n\n');
for k = 1:2
    ch = CH(k); sc = SC(k);
    if k == 1, fprintf('--- lateral  dn [m]\n'); else
               fprintf('--- heading  dalpha [deg]\n'); end
    fprintf('%7s | %9s %9s %9s %9s | %7s\n', 'held out', ...
            'no resid', 'no mass', 'with mass', 'shuffled', 'gain');
    for i = 1:nM
        e0 = err_raw(i,ch)*sc;
        e1 = mean(err(i,1,:,ch))*sc;
        e2 = mean(err(i,2,:,ch))*sc;
        e3 = mean(err(i,3,:,ch))*sc;
        fprintf('%6dt | %9.4f %9.4f %9.4f %9.4f | %+6.1f%%\n', ...
                mm(i), e0, e1, e2, e3, 100*(e1-e2)/e1);
    end
    d = err(:,1,:,ch) - err(:,2,:,ch);   d = d(:);
    fprintf('  pooled %d pairs: mean(no mass - with mass) %+.5f, ', numel(d), mean(d)*sc);
    fprintf('with-mass better in %d/%d\n\n', sum(d > 0), numel(d));
end

%% ---- Figure 1: 막대 + 시드 점 ----
figure('Color','w','Position',[60 60 1080 420])
for k = 1:2
    ch = CH(k); sc = SC(k);
    subplot(1,2,k); hold on; box off
    set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
            'GridColor',[.88 .88 .88],'GridAlpha',1,'Layer','bottom')

    % 보정 없음: 회색 배경 막대 (스케일 기준선)
    bar(x, err_raw(:,ch)*sc, 0.72, 'FaceColor', GREY, ...
        'EdgeColor','none', 'FaceAlpha', 0.30);

    hb = zeros(1,3);
    for a = 1:3
        v  = squeeze(err(:,a,:,ch))*sc;      % (적재, 시드)
        mu = mean(v, 2);
        hb(a) = bar(x + (a-2)*w, mu, w*0.9, 'FaceColor', COL(a,:), ...
                    'EdgeColor','none');
        for i = 1:nM       % 개별 시드를 점으로 - 산포를 숨기지 않는다
            plot(x(i) + (a-2)*w, v(i,:), 'o', 'MarkerSize', 3.5, ...
                 'MarkerFaceColor','w', 'MarkerEdgeColor', COL(a,:)*0.7, ...
                 'LineWidth', 0.8);
        end
    end

    set(gca,'XTick',x,'XTickLabel',arrayfun(@(m) sprintf('%dt',m), mm, ...
            'UniformOutput',false))
    xlabel('Held-out payload  (never seen in training)')
    ylabel(LBL{k})
    xlim([0.5 nM+0.5])
    title(TIT{k},'FontSize',10,'FontWeight','normal')
    if k == 1
        legend(hb, arm, 'Location','northwest','Box','off','FontSize',9);
    end
end

%% ---- Figure 2: 표 ----
figure('Color','w','Position',[80 80 900 420])
axis off
ty = 0.94;
text(0.02, ty, 'Leave-one-mass-out — prediction error on the unseen payload', ...
     'FontSize',12,'FontWeight','bold'); ty = ty - 0.055;
text(0.02, ty, ['3 seeds, mean' char(177) 'sd.  The payload column was removed ' ...
     'from training and validation.'], 'FontSize',9,'Color',[.35 .35 .35]);
ty = ty - 0.065;

for k = 1:2
    ch = CH(k); sc = SC(k);
    if k == 1, hd = 'Lateral  \Deltan  [m]'; else
               hd = 'Heading  \Delta\alpha  [deg]'; end
    text(0.02, ty, hd, 'FontSize',10,'FontWeight','bold'); ty = ty - 0.05;
    cols = {'held out','no residual','no mass','with mass','shuffled','gain'};
    cx   = [0.02 0.16 0.33 0.50 0.67 0.85];
    for c = 1:6
        text(cx(c), ty, cols{c}, 'FontSize',9,'FontWeight','bold', ...
             'Color',[.25 .25 .25]);
    end
    ty = ty - 0.012;
    line([0.02 0.96],[ty ty],'Color',[.75 .75 .75]); ty = ty - 0.042;
    for i = 1:nM
        v  = squeeze(err(i,:,:,ch))*sc;             % (팔, 시드)
        mu = mean(v,2);  sd = std(v,0,2);
        g  = 100*(mu(1)-mu(2))/mu(1);
        text(cx(1), ty, sprintf('%dt', mm(i)), 'FontSize',9);
        text(cx(2), ty, sprintf('%.4f', err_raw(i,ch)*sc), 'FontSize',9, ...
             'Color',[.45 .45 .45]);
        for a = 1:3
            text(cx(2+a), ty, sprintf('%.4f%c%.4f', mu(a), char(177), sd(a)), ...
                 'FontSize',9,'Color',COL(a,:)*0.75);
        end
        if g > 0, gc = RES*0.75; else gc = NOM*0.75; end
        text(cx(6), ty, sprintf('%+.1f%%', g), 'FontSize',9,'Color',gc);
        ty = ty - 0.042;
    end
    d = err(:,1,:,ch) - err(:,2,:,ch);  d = d(:);
    text(cx(1), ty, sprintf(['pooled %d pairs:  mean(no mass - with mass) ' ...
         '%+.5f,   with mass better in %d/%d'], numel(d), mean(d)*sc, ...
         sum(d>0), numel(d)), 'FontSize',9,'Color',[.35 .35 .35]);
    ty = ty - 0.075;
end
xlim([0 1]); ylim([0 1])
