function S = run_stats(name, T, kthr)
%RUN_STATS  runs/<name>.mat 을 읽어 lap_stats 통계를 돌려준다.
%   S = run_stats('blind_48_s1', T)            % 기본 kthr=0.020 (R<=50 m)
%   여러 개: S = run_stats({'a','b'}, T)  -> 구조체 배열
%
%   모든 그림이 같은 정의(1랩 컷 + 거리 가중 + 급코너 R<=50m)를 쓰도록 한 곳에 모음.

if nargin < 3 || isempty(kthr), kthr = 0.020; end
if ischar(name), name = {name}; end

S = struct('name', {}, 'mean', {}, 'rms', {}, 'corner', {}, 'max', {}, 'n_lap', {});
for i = 1:numel(name)
    f = fullfile('runs', [name{i} '.mat']);
    if ~exist(f, 'file')
        warning('run_stats:missing', '%s 없음 - 건너뜀', f);
        continue
    end
    r = load(f);
    q = lap_stats(r.s, r.n, T, kthr);
    S(end+1) = struct('name', name{i}, 'mean', q.mean, 'rms', q.rms, ...
                      'corner', q.corner_rms, 'max', q.max, 'n_lap', q.n_lap); %#ok<AGROW>
end
end
