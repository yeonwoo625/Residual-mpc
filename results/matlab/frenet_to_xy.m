function [X, Y] = frenet_to_xy(T, s, n)
%FRENET_TO_XY  (s, n) -> 전역 좌표 (X, Y).
%   reference_utils.frenet_to_cartesian 와 동일 규약:
%       X = Xr(s) - sin(psi_r)*n,   Y = Yr(s) + cos(psi_r)*n
%   즉 n > 0 이 경로 진행방향 기준 왼쪽.

sm  = mod(s(:), T.s_total);
xp  = interp1(T.s, T.x,   sm, 'linear', 'extrap');
yp  = interp1(T.s, T.y,   sm, 'linear', 'extrap');
psi = interp1(T.s, T.psi, sm, 'linear', 'extrap');

X = xp - sin(psi) .* n(:);
Y = yp + cos(psi) .* n(:);
end
