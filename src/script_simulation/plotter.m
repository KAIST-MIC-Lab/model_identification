function plotter(rec,model)
    %% FIGURE SETTING

    plot_opts.font_size = 16;
    plot_opts.line_width = 2;
    plot_opts.lgd_size = 16;
    plot_opts.fig_height = 200; 
    plot_opts.fig_width = 450;

    %% MAIN PLOT FUNCTIONS

    close all;
    garo = 1920;
    sero = 1080;
    chang = 80; 

    f_idx = 1;
    % figure(f_idx); clf;
    set(figure(f_idx), 'Position', [0, 0 - chang, garo/4, sero]);
    subplot(5,1,1);
    plot(rec.x_hat1.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.x_1.Data, 'r--', 'LineWidth', 2); hold on
    title('Figure 1: state 1');
    legend('x\_hat1', 'x\_1');
    grid on;
    subplot(5,1,2);
    plot(rec.x_hat2.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.x_2.Data, 'r--', 'LineWidth', 2); hold on
    title('Figure 2: state 2');
    legend('x\_hat2', 'x\_2');
    grid on;
    subplot(5,1,3);
    plot(rec.x_hat3.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.x_3.Data, 'r--', 'LineWidth', 2); hold on
    title('Figure 3: state 3');
    legend('x\_hat3', 'x\_3');
    grid on;
    subplot(5,1,4);
    plot(rec.x_hat4.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.x_4.Data, 'r--', 'LineWidth', 2); hold on
    title('Figure 4: state 4');
    legend('x\_hat4', 'x\_4');
    grid on;
    subplot(5,1,5);
    plot(rec.u1.Data, 'g-', 'LineWidth', 1); hold on
    plot(rec.u2.Data, 'b-', 'LineWidth', 2); hold on
    title('Figure 5: input');
    legend('u1', 'u2');
    grid on;
    f_idx = 2;

    % figure(f_idx); clf;
    set(figure(f_idx), 'Position', [0+1*garo/4, 0 - chang, garo/4, sero]);
    subplot(4,1,1);
    plot(rec.g_hat1.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.g1.Data, 'r--', 'LineWidth', 2); hold on
    title('approximation g_1');
    legend('g\_hat1', 'g\_1');
    grid on;
    subplot(4,1,2);
    plot(rec.g_hat2.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.g2.Data, 'r--', 'LineWidth', 2); hold on
    title('approximation g_2');
    legend('g\_hat2', 'g\_2');
    grid on;
    subplot(4,1,3);
    plot(rec.g_hat3.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.g3.Data, 'r--', 'LineWidth', 2); hold on
    title('approximation g_3');
    legend('g\_hat3', 'g\_3');
    grid on;
    subplot(4,1,4);
    plot(rec.g_hat4.Data, 'k-', 'LineWidth', 1); hold on
    plot(rec.g4.Data, 'r--', 'LineWidth', 2); hold on
    title('approximation g_4');
    legend('g\_hat4', 'g\_4');
    grid on;

    f_idx = 3;
    set(figure(f_idx), 'Position', [0+2*garo/4, 0 - chang, garo/4, sero]);
    N = length(rec.x_3.Time);           % 전체 데이터 길이
    half_idx = floor(N/2)+1;  % 절반부터 시작
    
    % Friction-velocity map for joint 1 (절반부터)
    subplot(3,1,1);
    plot(rec.x_3.Data(half_idx:end), rec.F_hat1.Data(half_idx:end), 'k-', 'MarkerSize', 1, 'DisplayName', 'Measured'); hold on
    plot(rec.x_3.Data(half_idx:end), rec.F1.Data(half_idx:end), 'r--', 'LineWidth', 2, 'DisplayName', 'Model');
    xlabel('q\_dot1 (rad/s)');
    ylabel('Friction (Nm)');
    title('Friction-Velocity Map: Joint 1');
    legend('show');
    grid on;

    % Friction-velocity map for joint 2 (절반부터)
    subplot(3,1,2);
    plot(rec.x_4.Data(half_idx:end), rec.F_hat2.Data(half_idx:end), 'k-', 'MarkerSize', 1, 'DisplayName', 'Measured'); hold on
    plot(rec.x_4.Data(half_idx:end), rec.F2.Data(half_idx:end), 'r--', 'LineWidth', 2, 'DisplayName', 'Model');
    xlabel('q\_dot2 (rad/s)');
    ylabel('Friction (Nm)');
    title('Friction-Velocity Map: Joint 2');
    legend('show');
    grid on;

    % subplot(3,1,3);
    % % plot(rec. V_Norm, 'g-', 'LineWidth', 1); hold on
    % plot(rec. W_Norm, 'b-', 'LineWidth', 2); hold on
    % xlabel('time');
    % title('Weight Norm');
    % legend( 'W\_Norm');
    % grid on;


end

%% LOCAL FUNCTION
function plot_cpr(fig_num, recs, data_info, plot_opts)
    fig_width = plot_opts.fig_width;
    fig_height = plot_opts.fig_height;
    line_width = plot_opts.line_width;
    font_size = plot_opts.font_size;
    lgd_size = plot_opts.lgd_size;
    
    start_time = plot_opts.start_time;
    end_time = plot_opts.end_time;
    
    figure(fig_num); clf; 
        hF = gcf; 
        hF.Position(3:4) = [fig_width, fig_height];

        for d_idx = 1:1:size(data_info,1)
            rec_name = data_info(d_idx,1);
            data_name = data_info(d_idx,2);
            color = data_info(d_idx,3);
            line_style = data_info(d_idx,4);

            Time = recs.(rec_name).(data_name).Time;
            Data = recs.(rec_name).(data_name).Data;

            obs_idx = find(Time >= start_time & Time <= end_time);
            
            plot(Time(obs_idx), Data(obs_idx), ...
                "Color", color, "LineWidth", line_width, "LineStyle", line_style); hold on
        end

        grid on; grid minor;

        xlabel(plot_opts.X_label, 'Interpreter', 'latex', 'FontSize', font_size);
        ylabel(plot_opts.y_label, 'Interpreter', 'latex', 'FontSize', font_size);

        % maxVal = max(rec1.Yaw_Ref(obs_idX1)); minVal = min(rec1.Yaw_Ref(obs_idX1)); 
        % len = maxVal-minVal; ratio = .1;
        % ylim([minVal-len*ratio maxVal+len*ratio]);
        % xlim([0 T])

        ax = gca;
        ax.FontSize = font_size; 
        ax.FontName = 'Times New Roman';
end