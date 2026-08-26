% figT7 - 잔차 미분을 넣으면 MPC가 믿는 "핸들 감도"가 요동친다
%
% 변수 5개가 전부입니다 (모두 152x1 벡터):
%   s           주행 거리 [m]
%   auth_off    미분 OFF 일 때 MPC가 믿는 조향 권한
%   auth_on     미분 ON  일 때 MPC가 믿는 조향 권한
%   jitter_off  미분 OFF 일 때 직전 스텝 대비 변화율 [%]
%   jitter_on   미분 ON  일 때 직전 스텝 대비 변화율 [%]

load('figT7_simple.mat')

figure('Color','w')

% --- 위: 조향 권한 ---
subplot(2,1,1)
plot(s, auth_off, 'g', 'LineWidth', 1.5); hold on
plot(s, auth_on,  'r', 'LineWidth', 1.5)
ylabel('조향 권한')
legend('미분 OFF (완주)', '미분 ON (발산)')
title('미분을 넣으면 MPC가 믿는 핸들 감도가 튄다')

% --- 아래: 스텝간 요동 ---
subplot(2,1,2)
plot(s, jitter_off, 'g', 'LineWidth', 1.5); hold on
plot(s, jitter_on,  'r', 'LineWidth', 1.5)
xlabel('주행 거리 s [m]')
ylabel('스텝간 변화율 [%]')
legend('미분 OFF', '미분 ON')

% --- 숫자 출력 ---
fprintf('미분 OFF : 스텝간 변화 %.1f%%\n', median(jitter_off,'omitnan'))
fprintf('미분 ON  : 스텝간 변화 %.1f%%\n', median(jitter_on, 'omitnan'))
fprintf('=> %.0f배 더 요동\n', median(jitter_on,'omitnan')/median(jitter_off,'omitnan'))
