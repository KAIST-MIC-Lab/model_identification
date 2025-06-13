% load('sim_results/logsout_org.mat');  % logsout이 포함된 .mat 파일

% 예시: logsout으로부터 원하는 데이터를 TimeSeries 형태로 추출
X_ts = logsout_org.get('X').Values;       
u_ts = logsout_org.get('u').Values;
Ref_ts = logsout_org.get('Ref').Values;

dt = 2e-3;
% Time and position data
time = X_ts.Time;                     % Nx1
q1 = X_ts.Data(1,:);  
q2 = X_ts.Data(1,:);  


% --- Simple numerical differentiation ---
qdot1_simple = [diff(q1)/dt, 0];  % same length as q1
qdot2_simple = [diff(q2)/dt, 0];

% --- Savitzky-Golay differentiation ---
poly_order = 1;
window_size = 3;

if exist('sgolayfilt', 'file')
    q1_sg = sgolayfilt(q1, poly_order, window_size);
    q2_sg = sgolayfilt(q2, poly_order, window_size);

    qdot1_diff_sg = gradient(q1_sg, dt);
    qdot2_diff_sg = gradient(q2_sg, dt);

    [~, g] = sgolay(poly_order, window_size);
    qdot1_diff_sg_direct = conv(q1, factorial(1) * g(:,2) / dt, 'same');
    qdot2_diff_sg_direct = conv(q2, factorial(1) * g(:,2) / dt, 'same');
else
    fprintf('Warning: Signal Processing Toolbox not available\n');
    qdot1_diff_sg = qdot1_simple;
    qdot2_diff_sg = qdot2_simple;
end

% === PLOT ===
figure('Name', 'SG Diff vs Simple Diff');

% Joint 1
subplot(2,1,1);
yyaxis left;
plot(time, q1, 'k-', 'LineWidth', 1.2); hold on;
ylabel('q_1 [rad]');
yyaxis right;
plot(time, qdot1_diff_sg, 'r-', 'LineWidth', 1.2);
plot(time, qdot1_simple, 'b--', 'LineWidth', 1.0);
ylabel('q̇_1 [rad/s]');
title('Joint 1: SG diff vs Simple diff');
legend('q_1 (raw)', 'q̇_1 (SG diff)', 'q̇_1 (simple)', 'Location', 'best');
grid on;
xlabel('Time [s]');

% Joint 2
subplot(2,1,2);
yyaxis left;
plot(time, q2, 'k-', 'LineWidth', 1.2); hold on;
ylabel('q_2 [rad]');
yyaxis right;
plot(time, qdot2_diff_sg, 'r-', 'LineWidth', 1.2);
plot(time, qdot2_simple, 'b--', 'LineWidth', 1.0);
ylabel('q̇_2 [rad/s]');
title('Joint 2: SG diff vs Simple diff');
legend('q_2 (raw)', 'q̇_2 (SG diff)', 'q̇_2 (simple)', 'Location', 'best');
grid on;
xlabel('Time [s]');

