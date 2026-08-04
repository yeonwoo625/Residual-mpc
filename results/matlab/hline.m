function h = hline(yv, col, txt)
%HLINE  가로 기준선 + 라벨 (yline 은 R2018b+ 라 구버전 호환용).
xl = xlim;
h = plot(xl, [yv yv], '--', 'Color', col, 'LineWidth', 1.2);
set(get(get(h,'Annotation'),'LegendInformation'), 'IconDisplayStyle', 'off');
if nargin > 2 && ~isempty(txt)
    text(xl(1) + 0.04*diff(xl), yv, txt, 'Color', col, ...
         'VerticalAlignment', 'bottom', 'FontSize', 8);
end
end
