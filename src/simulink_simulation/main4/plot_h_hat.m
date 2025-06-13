function plot_h_hat(logsout)
% plot_h_hat: Plot true h vs NN-identified h for multiple W/V snapshots
% Inputs:
%   logsout - Simulink logsout object containing 'X', 'u', 'h', 'W_hat', 'V_hat'

    % --- Data Extraction ---
    W_data = logsout.get('W_hat').Values.Data;
    V_data = logsout.get('V_hat').Values.Data;
    X_data = logsout.get('X').Values.Data;
    u_data = logsout.get('u').Values.Data;
    h_true = logsout.get('h').Values.Data;
    time   = logsout.get('X').Values.Time;

    n_time = length(time);
    idx_ratios = [100/130, 110/130, 120/130];
    titles_col = {'$\frac{90}{110}$', '$\frac{95}{110}$', '$\frac{100}{110}$'};
    titles_row = {'$h_1$', '$h_2$', '$h_3$', '$h_4$'};

    % --- Figure Setup ---
    close all;
    figure('Position', [0, -80, 3*1920/2, 1080]);

    for col = 1:3
        % Extract W, V at given snapshot
        target_idx = round(size(W_data, 3) * idx_ratios(col));
        W_sel = W_data(:, :, target_idx);
        V_sel = V_data(:, :, target_idx);

        % Recompute h_id over time
        h_id = zeros(4, n_time);
        for t = 1:n_time
            x = X_data(:, t);
            u = u_data(:, t);
            x_bar = [x; u];
            h_id(:, t) = h_hat_func(W_sel, V_sel, x_bar);
        end

        % Plot each hₖ row-wise
        for row = 1:4

            subplot(4, 3, (row - 1) * 3 + col);
            plot(time, h_true(row, :), 'k.', 'Markersize', 1); hold on;
            plot(time, h_id(row, :), 'r-', 'Markersize', 1);

            % Labels
            if row == 1
                title(['$W,V$ at ', titles_col{col}], 'Interpreter', 'latex');
            end
            if col == 1
                ylabel(titles_row{row}, 'Interpreter', 'latex');
            end
            if row >= 3
                ylim([-1.5, 1.5]);
            else
                ylim([-0.1, 0.1]);
            end
            grid on;
        end
    end

end

function h = h_hat_func(W, V, x_bar)
    h = W * tanh(V * x_bar);  % Ensure consistent sign convention
end
