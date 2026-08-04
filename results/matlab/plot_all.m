function plot_all()
%PLOT_ALL  모든 그림 스크립트를 한 번에 실행.
%   results/matlab/ 에서 실행할 것 (스크립트가 상대경로로 .mat 을 읽는다).
%   각 스크립트가 clear 를 호출하므로 별도 워크스페이스(run_one)에서 돌린다.

scripts = {'plot_fig1_track', 'plot_fig2_error_vs_s', 'plot_fig3_bars', ...
           'plot_fig4_mass_dependence', 'plot_fig9_ablation', 'plot_figM1_lambda'};
for i = 1:numel(scripts)
    fprintf('\n===== %s =====\n', scripts{i});
    run_one(scripts{i});
end
fprintf('\n완료: %d개 그림\n', numel(scripts));
end


function run_one(name)
run(name);
end
