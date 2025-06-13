%% Load data
load('LPF_on_60_100_20_6.mat');

% Extract time vector (convert to seconds if needed)
t = time_004 / 1000; % [ms] → [s]
fs = 500; % Sampling rate
dt = 0.002; % 실제 샘플링 시간 [s]

% Extract position q
q1 = data_2901(1, :);
q2 = data_2902(1, :);

% Extract raw qdot
qdot1_raw = data_2901(2, :);
qdot2_raw = data_2902(2, :);

% Extract filtered qdot
qdot1_filt = data_004(3, :);
qdot2_filt = data_004(4, :);



torque1_raw = data_2901(3, :);
torque2_raw = data_2902(3, :);

u1_raw = data_005(1, :);
u2_raw = data_005(2, :);

%% Improved Numerical differentiation of q with noise reduction

% Method 1: Simple differentiation (will be very noisy)
qdot1_diff_simple = [diff(q1)/dt, 0];
qdot2_diff_simple = [diff(q2)/dt, 0];

% Method 2: Pre-smoothing + differentiation
% Apply mild smoothing to position before differentiation
window_smooth = 5; % 5-point moving average
q1_smooth = smoothdata(q1, 'movmean', window_smooth);
q2_smooth = smoothdata(q2, 'movmean', window_smooth);

qdot1_diff_smooth = [diff(q1_smooth)/dt, 0];
qdot2_diff_smooth = [diff(q2_smooth)/dt, 0];

% Method 3: Discrete derivative with built-in smoothing
% Using gradient function (central difference with smoothing)
qdot1_diff_grad = gradient(q1, dt);
qdot2_diff_grad = gradient(q2, dt);

% Method 4: Savitzky-Golay differentiation (best for noisy position data)
poly_order = 1;
window_size = 35; % Larger window for more smoothing

if exist('sgolayfilt', 'file')
    % Pre-smooth position data
    q1_sg = sgolayfilt(q1, poly_order, window_size);
    q2_sg = sgolayfilt(q2, poly_order, window_size);
    
    % Then differentiate
    qdot1_diff_sg = gradient(q1_sg, dt);
    qdot2_diff_sg = gradient(q2_sg, dt);
    
    % Alternative: Direct derivative using SG coefficients
    [~, g] = sgolay(poly_order, window_size);
    qdot1_diff_sg_direct = conv(q1, factorial(1) * g(:,2) / dt, 'same');
    qdot2_diff_sg_direct = conv(q2, factorial(1) * g(:,2) / dt, 'same');
else
    fprintf('Warning: Signal Processing Toolbox not available\n');
    qdot1_diff_sg = qdot1_diff_grad;
    qdot2_diff_sg = qdot2_diff_grad;
end

% Method 5: Low-pass filter + differentiation
% Design a Butterworth low-pass filter
cutoff_freq = 50; % Hz - adjust based on your robot dynamics
nyquist = (1/dt)/2;
normalized_cutoff = cutoff_freq / nyquist;

if normalized_cutoff < 1
    [b_lpf, a_lpf] = butter(4, normalized_cutoff, 'low');
    q1_lpf = filtfilt(b_lpf, a_lpf, q1);
    q2_lpf = filtfilt(b_lpf, a_lpf, q2);
    
    qdot1_diff_lpf = gradient(q1_lpf, dt);
    qdot2_diff_lpf = gradient(q2_lpf, dt);
else
    qdot1_diff_lpf = qdot1_diff_grad;
    qdot2_diff_lpf = qdot2_diff_grad;
end

% Select the best method (Savitzky-Golay or LPF + gradient)
qdot1_diff = qdot1_diff_sg;
qdot2_diff = qdot2_diff_sg;

% ...existing code...

t0 = t(1); % 시작 시간

%% Compare: qdot_diff vs qdot_raw
figure('Name', 'Velocity Comparison: Raw vs Numerical Differentiation');
subplot(2,1,1);
plot(t - t0, qdot1_raw, 'k-', 'LineWidth', 1.5); hold on;
% plot(t - t0, qdot1_diff, 'b--', 'LineWidth', 1.2);
plot(t - t0, qdot1_filt, 'r:', 'LineWidth', 1.2);
grid on;
legend('Raw qdot', 'Filtered qdot', 'Location', 'best');
title('Joint 1: Velocity Comparison');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
xlim([0, t(end)-t0]); 
subplot(2,1,2);
plot(t - t0, qdot2_raw, 'k-', 'LineWidth', 1.5); hold on;
% plot(t - t0, qdot2_diff, 'b--', 'LineWidth', 1.2);
plot(t - t0, qdot2_filt, 'r:', 'LineWidth', 1.2);
grid on;
legend('Raw qdot', 'Filtered qdot', 'Location', 'best');
title('Joint 2: Velocity Comparison');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
xlim([0, t(end)-t0]); 
% ...existing code...

%% Plot raw q (position)
figure('Name', 'Raw Joint Position');
subplot(2,1,1);
plot(t - t0, q1, 'b-', 'LineWidth', 1.2);
grid on;
title('Joint 1: Raw Position');
xlabel('Time [s]');
ylabel('Position [rad]');
xlim([0, t(end)-t0]); 
subplot(2,1,2);
plot(t - t0, q2, 'r-', 'LineWidth', 1.2);
grid on;
title('Joint 2: Raw Position');
xlabel('Time [s]');
ylabel('Position [rad]');
xlim([0, t(end)-t0]); 


torque1 = data_2901(3, :);
torque2 = data_2902(3, :);

%% Plot raw torque
figure('Name', 'Raw Joint Torque');
subplot(2,1,1);
plot(t - t0, torque1, 'b-', 'LineWidth', 0.5);
grid on;
title('Joint 1: Raw Torque');
xlabel('Time [s]');
ylabel('Torque [Nm]');
xlim([0, t(end)-t0]); 
subplot(2,1,2);
plot(t - t0, torque2, 'r-', 'LineWidth', 1.2);
grid on;
title('Joint 2: Raw Torque');
xlabel('Time [s]');
ylabel('Troque[Nm]');
xlim([0, t(end)-t0]); 



%% Compare: qdot_diff vs qdot_raw
figure('Name', 'Velocity Comparison: Raw vs Filter qdot vs q');
subplot(2,1,1);
plot(t - t0, qdot1_raw, 'k-', 'LineWidth', 1.5); hold on;
plot(t - t0, q1, 'b-', 'LineWidth', 1.2);
plot(t - t0, qdot1_filt, 'r:', 'LineWidth', 1.2);
plot(t - t0, torque1, 'g-', 'LineWidth', 0.5); % torque1 추가
plot(t - t0, u1_raw, 'm--', 'LineWidth', 0.5); % u1_raw 추가
grid on;
legend('Raw qdot', 'Raw q', 'Filtered qdot', 'Torque', 'u','Location', 'best');
title('Joint 1: Velocity Comparison');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s] / Torque [Nm]');
xlim([0, t(end)-t0]); 

subplot(2,1,2);
plot(t - t0, qdot2_raw, 'k-', 'LineWidth', 1.5); hold on;
plot(t - t0, q2, 'r-', 'LineWidth', 1.2);
plot(t - t0, qdot2_filt, 'r:', 'LineWidth', 1.2);
plot(t - t0, torque2, 'g-', 'LineWidth', 0.5); % torque2 추가
plot(t - t0, u2_raw, 'm--', 'LineWidth', 0.5); % u2_raw 추가
grid on;
legend('Raw qdot', 'Raw q', 'Filtered qdot', 'Torque', 'u', 'Location', 'best');
title('Joint 2: Velocity Comparison');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s] / Torque [Nm]');
xlim([0, t(end)-t0]); 
