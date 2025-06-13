function check_PE_from_logsout(logsout, window_sec)
% check_PE_from_logsout: PE 조건을 logsout에서 직접 평가
% Inputs:
%   logsout    - Simulink logsout object
%   window_sec - sliding window 크기 [sec] (예: 1.0)

    % === Data Extraction ===
    W_data = logsout.get('W_hat').Values.Data;
    V_data = logsout.get('V_hat').Values.Data;
    X_data = logsout.get('X').Values.Data;
    u_data = logsout.get('u').Values.Data;
    time   = logsout.get('X').Values.Time;

    % === Settings ===
    dt = time(2) - time(1);
    T = window_sec;
    window_len = round(T / dt);
    N = length(time);

    % === NN 입력 phi = [x; u] ===
    phi_data = [X_data; u_data];  % [n_i x N]
    [n_i, ~] = size(phi_data);

    % === PE Evaluation ===
    min_eigs = zeros(1, N - window_len);
    cond_nums = zeros(1, N - window_len);

    for k = 1:(N - window_len)
        phi_win = phi_data(:, k:(k + window_len - 1));
        gram = phi_win * phi_win' * dt;  % approximate integral

        eig_vals = eig(gram);
        min_eigs(k) = min(eig_vals);
        cond_nums(k) = cond(gram);
    end

    % === Plotting ===
    figure('Name', 'PE Evaluation from logsout', 'Position', [100, 100, 800, 600]);

    subplot(2,1,1);
    plot(time(1:N - window_len), min_eigs, 'LineWidth', 1.5);
    ylabel('Min Eigenvalue of ∫φφᵗ');
    title(['PE Check: Sliding Window = ', num2str(window_sec), ' sec']);
    grid on;

    subplot(2,1,2);
    plot(time(1:N - window_len), cond_nums, 'LineWidth', 1.5);
    ylabel('Condition Number of Gramian');
    xlabel('Time (s)');
    grid on;
end
