function idx = first_lap(s, s_total)
%FIRST_LAP  s 가 되감기는 지점 전까지의 인덱스.
%   미세한 감소(수치 노이즈, 순간 후진)는 무시하고 큰 음의 점프만 랩 경계로 본다.
%   Hockenheim 주행은 모두 1랩 + 잔여 3~5% 라 이 컷이 필요하다.

s = s(:);
w = find(diff(s) < -0.5*s_total, 1, 'first');
if isempty(w)
    idx = find(s <= s_total);           % 단조 증가형이면 s_total 로 컷
else
    idx = (1:w)';
end
end
