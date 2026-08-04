function plot_all_payload()
%PLOT_ALL_PAYLOAD  적재 조건화 실험 그림 A~D 를 한 번에.
%   results/matlab/ 에서 실행 (상대경로로 runs/*.mat 을 읽는다).
%
%   A  plot_figA_allloads    전 적재에서 무게-blind 잔차가 작동
%   B/C plot_figB_heldout56  미지 적재 56t, blind vs 조건화
%   D  plot_figC_placebo     가짜 무게 플라시보 (조건화 이득의 정체)
%   +  plot_fig10_heldout    개루프 held-out R^2 (heldout/heldout_mass.mat)

scripts = {'plot_figA_allloads', 'plot_figB_heldout56', ...
           'plot_figC_placebo', 'plot_fig10_heldout'};
for i = 1:numel(scripts)
    fprintf('\n===== %s =====\n', scripts{i});
    run_one(scripts{i});
end
fprintf('\n완료: %d개 그림\n', numel(scripts));
end


function run_one(name)
run(name);
end
