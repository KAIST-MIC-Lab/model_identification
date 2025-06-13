function main_plotter4_1(logsout)
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
    
    Ref_ts = logsout.get("Ref");

    X_ts = logsout.get('X');    
    X_hat_ts = logsout.get('X_hat');
    u_ts = logsout.get('u');

    h_ts = logsout.get('h');
    h_hat_ts = logsout.get('h_hat');

    F_ts = logsout.get('F');
    F_hat_ts = logsout.get('F_hat');

    Norm_ts = logsout.get('Weight_Norm');

    % 데이터 추출
    X_data = X_ts.Values.Data;
    X1     = X_data(1,:);   
    X2     = X_data(2,:);   
    X3     = X_data(3,:);   
    X4     = X_data(4,:);   
    time = X_ts.Values.Time;  

    Ref_data = Ref_ts.Values.Data;
    Ref1     = Ref_data(1,:);   
    Ref2     = Ref_data(2,:);   
    Ref3     = Ref_data(3,:);   
    Ref4     = Ref_data(4,:);      

    X_hat_data = X_hat_ts.Values.Data;
    X_hat1     = X_hat_data(1,:);   
    X_hat2     = X_hat_data(2,:);   
    X_hat3     = X_hat_data(3,:);   
    X_hat4     = X_hat_data(4,:);  

    u_data = u_ts.Values.Data;
    u1     = u_data(1,:);  
    u2     = u_data(2,:);   

    h_data = h_ts.Values.Data;
    h1     = h_data(1,:);  
    h2     = h_data(2,:);  
    h3     = h_data(3,:);  
    h4     = h_data(4,:);  

    h_hat_data = h_hat_ts.Values.Data;
    h_hat1     = h_hat_data(1,:);  
    h_hat2     = h_hat_data(2,:);  
    h_hat3     = h_hat_data(3,:);  
    h_hat4     = h_hat_data(4,:);  

    F_data = F_ts.Values.Data;
    F1     = F_data(1,:);  
    F2     = F_data(2,:);  

    F_hat_data = F_hat_ts.Values.Data;
    F_hat1 = F_hat_data(1,:);   
    F_hat2 = F_hat_data(2,:);    

    Norm_data = Norm_ts.Values.Data;
    W_Norm     = Norm_data(2,:);
    V_Norm     = Norm_data(1,:);

    close all;
    garo = 2400;
    sero = 1000;
    chang = 80; 
 
    % === Trajectory 1 ===
    q0 = [-pi/2; 0];
    qd1 = [pi/4; pi/2];
    qd2 = [pi/4 + pi/2; -pi/2];
    waypoints = [q0, ... 
                 qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, ...
                 qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, ...
                 qd1, q0];
    traj_duration = 4.0;
    init_duration = 4.0;
    end_duration = 4.0;


    % === 공통 처리 ===
    num_segments = size(waypoints, 2) - 1;
    
    traj_times = traj_duration * ones(1, num_segments);
    traj_times(1) = init_duration;
    traj_times(end) = end_duration;
    
    Tidle = 1.0;

    % Compute total time
    Ttotal = Tidle * (num_segments + 1) + sum(traj_times);

    
    % 전체 시뮬레이션 시간 벡터
    N = length(X3);
    t_total = linspace(0, Ttotal, N);
    
    % 중간 구간 개수
    num_middle = num_segments - 2;
    
    % 중간 trajectory의 시작 시점
    time_marker = Tidle + traj_times(1) + Tidle/2;
    
    % 전체 중간 trajectory 시간
    middle_time = sum(traj_times(2:end-1)) + num_middle * Tidle;
    
    % 3등분된 각 길이
    len_third = middle_time / 3;
    
    % 각 구간의 시간 범위 설정 및 인덱스 계산
    start_1 = time_marker;
    end_1   = start_1 + len_third;
    idx_1   = find(t_total >= start_1 & t_total < end_1);
    
    start_2 = end_1;
    end_2   = start_2 + len_third;
    idx_2   = find(t_total >= start_2 & t_total < end_2);
    
    start_3 = end_2;
    end_3   = start_3 + len_third;
    idx_3   = find(t_total >= start_3 & t_total < end_3);


    close all;
    garo = 2400;
    sero = 1000;
    chang = 80;
    
    f_idx = 1;
    fig = figure(f_idx); clf;
    set(fig, 'Position', [0, -chang, garo, sero]);
    
    % 4x4 tiled layout
    tl = tiledlayout(4, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % --- h1 (1,1) ---
    nexttile(tl, 1);
    plot(time, h_hat1, 'k-', time, h1, 'r--', 'LineWidth', 1);
    title('approximation h_1'); legend('$\hat{h}_1$', '$h_1$', 'Interpreter', 'latex'); grid on;
    ylim([-0.04 0.04]);
    
    % --- h2 (1,2) ---
    nexttile(tl, 2);
    plot(time, h_hat2, 'k-', time, h2, 'r--', 'LineWidth', 1);
    title('approximation h_2'); legend('$\hat{h}_2$', '$h_2$', 'Interpreter', 'latex'); grid on;
    ylim([-0.04 0.04]);
    
    % --- h3 (2,1~2) ---
    nexttile(tl, 5, [1 2]);
    plot(time, h_hat3, 'k-', time, h3, 'r--', 'LineWidth', 1);
    title('approximation h_3'); legend('$\hat{h}_3$', '$h_3$', 'Interpreter', 'latex'); grid on;
    ylim([-1.5 1.5]);
    
    % --- h4 (3,1~2) ---
    nexttile(tl, 9, [1 2]);
    plot(time, h_hat4, 'k-', time, h4, 'r--', 'LineWidth', 1);
    title('approximation h_4'); legend('$\hat{h}_4$', '$h_4$', 'Interpreter', 'latex'); grid on;
    ylim([-1.5 1.5]);
    
    % --- Norm (4,1~2) ---
    nexttile(tl, 13, [1 2]);
    plot(time, V_Norm, 'g-', 'LineWidth', 1); hold on;
    plot(time, W_Norm, 'b-', 'LineWidth', 2);
    ylabel('Norm Value'); xlabel('Time (s)');
    title('Weight Norm');
    legend('V\_Norm', 'W\_Norm', 'Location', 'northwest'); grid on;
    ylim([0 5]);
    
    % --- Joint 1 (Full) (1,3~4) ---
    nexttile(tl, 3, [1 2]);
    plot(X3(idx_3), F1(idx_3), 'k.', ...
         X3(idx_1), F_hat1(idx_1), 'r.', ...
         X3(idx_2), F_hat1(idx_2), 'y.', ...
         X3(idx_3), F_hat1(idx_3), 'g.', 'MarkerSize', 1);
    xlabel('$\dot{q}_1$', 'Interpreter', 'latex'); ylabel('Friction (Nm)');
    title('Friction-Velocity Joint 1 (Full)');
    xlim([-0.4 0.4]); ylim([-2 2]); legend('show', 'Location', 'southeast'); grid on;
    
    % --- Joint 2 (Full) (2,3~4) ---
    nexttile(tl, 7, [1 2]);
    plot(X4(idx_3), F2(idx_3), 'k.', ...
         X4(idx_1), F_hat2(idx_1), 'r.', ...
         X4(idx_2), F_hat2(idx_2), 'y.', ...
         X4(idx_3), F_hat2(idx_3), 'g.', 'MarkerSize', 1);
    xlabel('$\dot{q}_2$', 'Interpreter', 'latex'); ylabel('Friction (Nm)');
    title('Friction-Velocity Joint 2 (Full)');
    xlim([-2 2]); ylim([-2 2]); legend('show', 'Location', 'southeast'); grid on;
    
    % --- Joint 1 (Zoom) (3,3~4) ---
    nexttile(tl, 11, [1 2]);
    plot(X3(idx_3), F1(idx_3), 'k.', ...
         X3(idx_1), F_hat1(idx_1), 'r.', ...
         X3(idx_2), F_hat1(idx_2), 'y.', ...
         X3(idx_3), F_hat1(idx_3), 'g.', 'MarkerSize', 1);
    xlabel('$\dot{q}_1$', 'Interpreter', 'latex'); ylabel('Friction (Nm)');
    title('Friction-Velocity Joint 1 (Zoom)');
    xlim([-0.03 0.03]); ylim([-1.2 1.2]); legend('show', 'Location', 'southeast'); grid on;
    
    % --- Joint 2 (Zoom) (4,3~4) ---
    nexttile(tl, 15, [1 2]);
    plot(X4(idx_3), F2(idx_3), 'k.', ...
         X4(idx_1), F_hat2(idx_1), 'r.', ...
         X4(idx_2), F_hat2(idx_2), 'y.', ...
         X4(idx_3), F_hat2(idx_3), 'g.', 'MarkerSize', 1);
    xlabel('$\dot{q}_2$', 'Interpreter', 'latex'); ylabel('Friction (Nm)');
    title('Friction-Velocity Joint 2 (Zoom)');
    xlim([-0.03 0.03]); ylim([-0.8 0.8]); legend('show', 'Location', 'southeast'); grid on;
    
    % ==== Figure 2 ====
    f_idx = 2;
    fig = figure(f_idx); clf;
    set(fig, 'Position', [0+50, -chang, garo, sero]);
    
    % 4x4 tiled layout
    t2 = tiledlayout(4, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % === State 1 ===
    nexttile(t2, 1, [1 2]);
    plot(time, X_hat1, 'k-', 'LineWidth', 1); hold on;
    plot(time, X1, 'k--', 'LineWidth', 1); hold on;
    plot(time(idx_1), Ref1(idx_1), 'r--', 'LineWidth', 1); hold on;
    plot(time(idx_2), Ref1(idx_2), 'Color', [1 0.7 0.3], 'LineStyle', '--', 'LineWidth', 1); hold on;
    plot(time(idx_3), Ref1(idx_3), 'g--', 'LineWidth', 1); hold on;
    title('State 1'); legend('$\hat{x}_1$', '$x_1$', '$r_1$', 'Interpreter', 'latex');
    ylabel('State'); grid on;
    
    % === State 2 ===
    nexttile(t2, 5, [1 2]);
    plot(time, X_hat2, 'k-', 'LineWidth', 1); hold on;
    plot(time, X2, 'k--', 'LineWidth', 1); hold on;
    plot(time(idx_1), Ref2(idx_1), 'r--', 'LineWidth', 1); hold on;
    plot(time(idx_2), Ref2(idx_2), 'Color', [1 0.7 0.3], 'LineStyle', '--', 'LineWidth', 1); hold on;
    plot(time(idx_3), Ref2(idx_3), 'g--', 'LineWidth', 1); hold on;
    title('State 2'); legend('$\hat{x}_2$', '$x_2$', '$r_2$', 'Interpreter', 'latex');
    ylabel('State'); grid on;
    
    % === State 3 ===
    nexttile(t2, 9, [1 2]);
    plot(time, X_hat3, 'k-', 'LineWidth', 1); hold on;
    plot(time, X3, 'k--', 'LineWidth', 1); hold on;
    plot(time(idx_1), Ref3(idx_1), 'r--', 'LineWidth', 1); hold on;
    plot(time(idx_2), Ref3(idx_2), 'Color', [1 0.7 0.3], 'LineStyle', '--', 'LineWidth', 1); hold on;
    plot(time(idx_3), Ref3(idx_3), 'g--', 'LineWidth', 1); hold on;
    title('State 3'); legend('$\hat{x}_3$', '$x_3$', '$r_3$', 'Interpreter', 'latex');
    ylabel('State'); grid on;
    
    % === State 4 ===
    nexttile(t2, 13, [1 2]);
    plot(time, X_hat4, 'k-', 'LineWidth', 1); hold on;
    plot(time, X4, 'k--', 'LineWidth', 1); hold on;
    plot(time(idx_1), Ref4(idx_1), 'r--', 'LineWidth', 1); hold on;
    plot(time(idx_2), Ref4(idx_2), 'Color', [1 0.7 0.3], 'LineStyle', '--', 'LineWidth', 1.5); hold on;
    plot(time(idx_3), Ref4(idx_3), 'g--', 'LineWidth', 1); hold on;
    title('State 4'); legend('$\hat{x}_4$', '$x_4$', '$r_4$', 'Interpreter', 'latex');
    ylabel('State'); xlabel('Time (s)'); grid on;


        
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