% 시드 간 재현성 - 값은 식별되고 미분은 식별되지 않는다
%
% 메시지: 같은 데이터/구조로 초기값만 바꿔 학습한 모델들이
%         값은 거의 같고(3%), 미분은 제각각(67%)이다.
%         → 미분은 트럭의 물리가 아니라 학습의 우연이다. 그래서 주입하면 발산한다.
%
% 색: 파랑(값) / 주황(미분). 적록색약 안전. 초록/빨강 쓰지 말 것.

load('fig_seed_jacobian.mat')

VAL  = [0.12 0.47 0.71];      % 파랑 - 값
JAC  = [0.90 0.49 0.13];      % 주황 - 미분
y    = [dn_value, dn_jac(:)'];
lbl  = {'\Deltan', ...
        '\partial/\partialn', '\partial/\partial\alpha', '\partial/\partialv', ...
        '\partial/\partialD', '\partial/\partial\delta', '\partial/\partial\kappa'};

figure('Color','w','Position',[100 100 660 420]); hold on; box off
set(gca,'FontSize',11,'TickDir','out','YGrid','on', ...
        'GridColor',[.85 .85 .85],'GridAlpha',1,'Layer','bottom')

for i = 1:numel(y)
    c = VAL; if i > 1, c = JAC; end
    bar(i, y(i), 0.6, 'FaceColor', c, 'EdgeColor','none');
    text(i, y(i)+2.5, sprintf('%.0f%%', y(i)), 'HorizontalAlignment','center', ...
         'FontSize',10, 'FontWeight','bold');
end

% 값 / 미분 구분선
plot([1.5 1.5], [0 100], ':', 'Color',[.6 .6 .6], 'LineWidth',1);
text(1,   -11, 'Value',      'HorizontalAlignment','center', ...
     'FontSize',11,'FontWeight','bold','Color',VAL);
text(4.5, -11, 'Derivative', 'HorizontalAlignment','center', ...
     'FontSize',11,'FontWeight','bold','Color',JAC);

set(gca,'XTick',1:numel(y),'XTickLabel',lbl,'XLim',[0.4 numel(y)+0.6], ...
        'YLim',[0 max(y)*1.2])
ylabel('Disagreement across seeds  [%]')

h(1) = patch(nan,nan,VAL,'EdgeColor','none');
h(2) = patch(nan,nan,JAC,'EdgeColor','none');
legend(h, {'Residual value  \Deltan', 'State Jacobian  \partial\Deltan/\partialx'}, ...
       'Location','northwest','Box','off','FontSize',10);

fprintf('\n시드 %d개 / 샘플 %d개\n', n_seeds, n_samples);
fprintf('  값   Δn        : %.1f%%\n', dn_value);
fprintf('  미분 ∂Δn/∂x 평균: %.1f%%   →  %.0f배\n\n', ...
        mean(dn_jac), mean(dn_jac)/dn_value);
