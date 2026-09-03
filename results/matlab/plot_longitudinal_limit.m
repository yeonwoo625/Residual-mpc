% 종방향 잔차를 쓰지 않은 근거 - 기어 변속 때문에 학습이 불가능하다
%
% 배포 설정(B_MATRIX)은 잔차의 횡방향(Δn, Δα)만 쓰고 종방향(Δs, Δv)은 버린다.
% 그 근거를 세 갈래로 보인다.
%
%   (a) 변속 흔적    스로틀이 D=1.00 으로 고정인데 가속도가 반복적으로 붕괴한다.
%                    토크 단절 = 변속. 정지 출발 구간에서 8회 검출.
%   (b) 잔차 집중    |Δv| 상위 1% 샘플이 그 속도대(3~8 m/s)에 15~30배 몰린다.
%                    순항(11~12 m/s, 전체의 90%)에는 하나도 없다.
%   (c) 학습 불가    (v, D) 를 좁게 묶어도 Δv 는 크게 흩어진다. 같은 구간 Δn 대비
%                    최대 437배. 변속기 상태가 입력에 없어 모델이 구분할 수 없다.
%
% 노이즈 실링(설명 불가 비율)도 같은 결론이다: 전 구간 Δn 1.6% vs Δv 50.1%,
% 그러나 변속이 없는 순항 구간만 보면 Δv 가 12.8% 로 떨어진다.
%
% 데이터: fig_longitudinal_limit.mat  (results/longitudinal_limit.py 가 생성)
% 실행:   results/matlab/ 폴더 안에서 `plot_longitudinal_limit`
% 요구:   base MATLAB 만 (툴박스 불필요). R2013a 이상.

clear; close all
load('fig_longitudinal_limit.mat')

NOM  = [0.12 0.47 0.71];
RES  = [0.90 0.49 0.13];
GREY = [0.55 0.55 0.55];

%% ---- 수치 표 (명령창) ----
fprintf('\n=== Gear shifts during launch (throttle held at D = 1.00) ===\n');
fprintf('%8s %9s %7s %11s\n','t [s]','v [m/s]','D','acc [m/s2]');
for k = 1:numel(shift_idx)
    j = shift_idx(k);
    fprintf('%8.1f %9.2f %7.2f %11.3f\n', t(j), v_launch(j), D_launch(j), acc_launch(j));
end
fprintf('\n=== Residual spread within narrow (v, D) bins ===\n');
fprintf('%12s %7s %13s %13s %8s\n','v [m/s]','n','std dv','std dn','ratio');
for k = 1:size(spread,1)
    fprintf('%12.1f %7d %13.4f %13.4f %7.0fx\n', spread(k,1), round(spread(k,2)), ...
            spread(k,3), spread(k,4), spread(k,3)/max(spread(k,4),1e-9));
end
fprintf('\n=== Unexplained variance (k-NN noise floor) ===\n');
fprintf('%18s %10s %10s\n','', 'dn (lat)', 'dv (long)');
fprintf('%18s %9.1f%% %9.1f%%\n','all speeds', 100*floor_all(2), 100*floor_all(4));
fprintf('%18s %9.1f%% %9.1f%%\n','cruise 11.5-12.5', 100*floor_cruise(2), 100*floor_cruise(4));
fprintf('\n');

figure('Color','w','Position',[60 60 1280 380])

%% (a) 변속 흔적 - 스로틀 고정인데 가속도가 무너진다
subplot(1,3,1); hold on; box off
set(gca,'FontSize',10,'TickDir','out','XGrid','on','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
h1 = plot(t, acc_launch, '-', 'Color', NOM, 'LineWidth', 1.4);
h2 = plot(t, D_launch,   '-', 'Color', GREY, 'LineWidth', 1.6);
h3 = plot(t(shift_idx), acc_launch(shift_idx), 'v', 'Color', RES, ...
          'MarkerSize', 7, 'MarkerFaceColor', RES);
legend([h2 h1 h3], {'throttle  D','acceleration  [m/s^2]','shift detected'}, ...
       'Location','northeast','Box','off','FontSize',8.5);
xlim([0 max(t)]); ylim([-0.15 1.75])
xlabel('Time from standstill  [s]'); ylabel('D  /  acceleration  [m/s^2]')
title('(a)  Throttle held, acceleration collapses','FontSize',11,'FontWeight','normal')

%% (b) |Δv| 상위 1% 가 몰리는 속도
subplot(1,3,2); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
bar(bin_center, concentration, 0.75, 'FaceColor', RES, 'EdgeColor','none');
plot([min(bin_center)-0.6 max(bin_center)+0.6], [1 1], '--', ...
     'Color', GREY, 'LineWidth', 1.4);
text(max(bin_center), 2.6, 'evenly spread', 'HorizontalAlignment','right', ...
     'FontSize',9,'Color',GREY);
% 순항 구간 표시 - 전체 샘플의 90% 가 여기 있는데 상위 1% 는 0 개다
[~, ic] = max(bin_count);
text(bin_center(ic), 3.2, sprintf('%.0f%% of data\nzero spikes', ...
     100*bin_count(ic)/sum(bin_count)), 'HorizontalAlignment','center', ...
     'FontSize',8.5,'Color',[0.3 0.3 0.3]);
xlim([min(bin_center)-0.6 max(bin_center)+0.6]); ylim([0 32])
xlabel('Speed  [m/s]'); ylabel('Concentration of top-1% |\Deltav|')
title('(b)  Longitudinal spikes sit at shift speeds','FontSize',11,'FontWeight','normal')

%% (c) 같은 입력, 다른 정답
subplot(1,3,3); hold on; box off
set(gca,'FontSize',10,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')
x = (1:size(spread,1))';  w = 0.34;
b1 = bar(x-w/2, spread(:,3), w, 'FaceColor', RES, 'EdgeColor','none');
b2 = bar(x+w/2, spread(:,4), w, 'FaceColor', NOM, 'EdgeColor','none');
for k = 1:numel(x)
    text(x(k), spread(k,3)*1.5, sprintf('%.0fx', spread(k,3)/max(spread(k,4),1e-9)), ...
         'HorizontalAlignment','center','FontSize',9,'Color',[0.3 0.3 0.3]);
end
legend([b1 b2], {'\Deltav  (longitudinal)','\Deltan  (lateral)'}, ...
       'Location','southwest','Box','off','FontSize',9);
set(gca,'YScale','log','XTick',1:numel(x),'XTickLabel', ...
        arrayfun(@(a) sprintf('%.1f', a), spread(:,1), 'UniformOutput', false))
xlim([0.4 numel(x)+0.6]); ylim([5e-5 1])
xlabel('Speed bin centre  [m/s]   (D > 0.9)')
ylabel('Residual std within the bin')
title('(c)  Same input, scattered answer','FontSize',11,'FontWeight','normal')
