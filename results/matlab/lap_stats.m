function S = lap_stats(s, n, T, kthr, ds)
%LAP_STATS  1랩 컷 + 거리 가중(s 균일 재샘플링) 횡오차 통계.
%   모든 그림이 같은 숫자를 쓰도록 정의를 여기 한 곳에 모은다.
%
%   S = lap_stats(traj.s, traj.n, T)              기본 kthr=0.020 (R<=50 m), ds=1 m
%   S = lap_stats(traj.s, traj.n, T, 0.010)       급코너 R<=100 m
%
%   반환 필드
%     S.s, S.v        1 m 간격 재샘플링된 s 와 |n|
%     S.corner        급코너 논리 인덱스 (|kappa| >= kthr)
%     S.mean, S.rms   랩 전체 평균 / RMS      <- 거리 가중
%     S.corner_mean, S.corner_rms             <- 급코너 구간
%     S.max           최대 |n|
%     S.n_raw, S.n_lap  원본 / 1랩 샘플 수
%
%   시간 가중(원시 샘플 평균)이 아니라 거리 가중인 이유: 코너에서 속도가 느려
%   시간 샘플이 몰리면 코너 오차가 과대대표된다.

if nargin < 4 || isempty(kthr), kthr = 0.020; end     % R <= 50 m
if nargin < 5 || isempty(ds),   ds   = 1.0;   end     % 재샘플 간격 [m]

k  = first_lap(s, T.s_total);
sl = s(k);  nl = abs(n(k));

% 데이터 범위 안에서만 재샘플 (외삽하면 값이 폭주한다)
sg = (ceil(sl(1)) : ds : floor(sl(end)))';
v  = interp1(sl, nl, sg);

kap = abs(interp1(T.s, T.kappa, mod(sg, T.s_total), 'linear', 'extrap'));
cm  = kap >= kthr;

S = struct('s', sg, 'v', v, 'corner', cm, 'kthr', kthr, ...
           'mean', mean(v), 'rms', sqrt(mean(v.^2)), ...
           'corner_mean', mean(v(cm)), 'corner_rms', sqrt(mean(v(cm).^2)), ...
           'max', max(v), 'n_raw', numel(s), 'n_lap', numel(k));
end
