function v = pctl(x, q)
%PCTL  분위수 (Statistics Toolbox 없이). MATLAB quantile 과 같은 선형보간 규약.
%   v = pctl(abs(kappa), 0.8)   % 상위 20% 경계

x = sort(x(:)); n = numel(x);
if n == 1, v = x; return; end
pos = q*n + 0.5;
v = interp1(1:n, x, min(max(pos, 1), n), 'linear');
end
