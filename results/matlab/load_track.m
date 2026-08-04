function T = load_track(matfile)
%LOAD_TRACK  기준 경로를 읽어 호장길이 s, 접선각 psi, 곡률 kappa를 만든다.
%   T = load_track()                      % 기본: Hockenheim 1랩
%   T = load_track('waypoints/ref_waypoints.mat')
%
%   T.s, T.x, T.y, T.psi, T.kappa, T.s_total
%   kappa는 웨이포인트 간격(평균 2 m)의 수치미분이라 15점 이동평균으로 평활화.

if nargin < 1, matfile = 'waypoints/hockenheim_1lap.mat'; end
W = load(matfile);
x = W.x(:); y = W.y(:);

% 호장길이
s = [0; cumsum(hypot(diff(x), diff(y)))];

% 접선각 (unwrap 해야 interp1이 -pi/pi 경계에서 안 튄다)
psi = unwrap(atan2(gradient(y), gradient(x)));

% 곡률 kappa = (x'y'' - y'x'') / (x'^2+y'^2)^{3/2},  ' = d/ds
dx  = gradient(x, s);   dy  = gradient(y, s);
ddx = gradient(dx, s);  ddy = gradient(dy, s);
kappa = (dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;
kappa = smooth_ma(kappa, 15);

T = struct('s', s, 'x', x, 'y', y, 'psi', psi, 'kappa', kappa, 's_total', s(end));
end


function ys = smooth_ma(y, w)
% 이동평균 (툴박스 없이). 양 끝은 짧은 창으로 처리.
y = y(:); n = numel(y); ys = y;
h = floor(w/2);
for i = 1:n
    lo = max(1, i-h); hi = min(n, i+h);
    ys(i) = mean(y(lo:hi));
end
end
