function [m, thr] = corner_mask(T, s, varargin)
%CORNER_MASK  급코너 구간 논리 인덱스. 네 가지 정의를 지원한다.
%
%   corner_mask(T, s)                    상대: 곡률 상위 20% (기본, 기존 동작)
%   corner_mask(T, s, 0.95)              상대: 곡률 상위 5%
%   corner_mask(T, s, 'kappa', 0.020)    절대: |kappa| >= 0.020 [1/m]
%   corner_mask(T, s, 'radius', 50)      절대: 곡률반경 <= 50 m   (= kappa 0.020)
%   corner_mask(T, s, 'alat', 2.8, 12)   절대: v^2*|kappa| >= 2.8 [m/s^2] @ v=12
%
%   [m, thr] = ... 는 실제 적용된 |kappa| 임계값도 돌려준다.
%
%   Hockenheim 참고:  상위20% = 0.0110 (R 91 m),  상위5% = 0.0195 (R 51 m),
%                     최대 0.0260 (R 38 m).

if isempty(varargin)
    mode = 'quantile';  val = 0.8;
elseif isnumeric(varargin{1})
    mode = 'quantile';  val = varargin{1};
else
    mode = lower(varargin{1});  val = varargin{2};
end

switch mode
    case 'quantile'
        thr = pctl(abs(T.kappa), val);
    case 'kappa'
        thr = val;
    case 'radius'
        thr = 1/val;
    case 'alat'
        if numel(varargin) < 3
            error('corner_mask:alat', "'alat' 는 속도도 필요: corner_mask(T,s,'alat',2.8,12)");
        end
        thr = val / varargin{3}^2;
    otherwise
        error('corner_mask:mode', '알 수 없는 정의: %s', mode);
end

ks = interp1(T.s, T.kappa, mod(s(:), T.s_total), 'linear', 'extrap');
m  = abs(ks) >= thr;
end
