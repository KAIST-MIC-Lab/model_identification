clear

%% 
rec1_name = "30-Apr-2025_20-46-35";
rec2_name = "30-Apr-2025_20-44-54";

%% FIGURE SETTING
POSITION_FLAG = 1; % it will plot fiugures in the same position

plot_opts.font_size = 16;
plot_opts.line_width = 2;
plot_opts.lgd_size = 16;
plot_opts.fig_height = 200; 
plot_opts.fig_width = 450;

% For papers
% font_size = 32;
% line_width = 2;
% lgd_size = 28;
% fig_height = 300; 
% fig_width = 800;

%%
rec1 = load("results/" + rec1_name + ".mat");
rec2 = load("results/" + rec2_name + ".mat");

recs.rec1 = rec1.rec;
recs.rec2 = rec2.rec;

%% MAIN PLOT FUNCTIONS
plot_opts.start_time = 0; % start time
plot_opts.end_time = 10; % end time

%% BIRD'S EYE VIEW
f_idx = 1;
figure(f_idx);clf;
plot(recs.rec2.X.Data,      recs.rec2.Y.Data, 'cyan', 'LineWidth', 2); hold on
plot(recs.rec1.X.Data,      recs.rec1.Y.Data, 'b', 'LineWidth', 2); hold on
plot(recs.rec1.X_ref.Data,  recs.rec1.Y_ref.Data, 'r', 'LineWidth', 2, 'LineStyle', '--'); hold on


f_idx = f_idx+1;

%% PLOT SELECTED DATA
data_names = ["x_dot", "delta_f", "control_delta_f", "delta_r", "control_delta_r"];

for data_name = data_names
    data_info = [...
        "rec2", data_name, "cyan", "--"; ...
        "rec1", data_name, "blue", "-." ...
        ];
    plot_opts.x_label = "Time / s";
    plot_opts.y_label = "$\delta$ / rad";
    % plot_opts.y_label = data_name;

    plot_opts.LGD_SHOW = false;
    plot_cpr(f_idx, recs, data_info, plot_opts);
    f_idx = f_idx+1;
end

% %% Fig. 1: State 1 (Ref vs Obs)
data_info = [...
    "rec1", data_name, "red", "--"; ...
    "rec2", data_name, "blue", "-" ...
    ];
plot_opts.x_label = "Time / s";
plot_opts.y_label = "$\psi$ / rad";

plot_opts.LGD_SHOW = false;
plot_cpr(7, recs, data_info, plot_opts);

%%  




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
            "Color", color, "LineWidth", line_width, "LineStyle", line_style); 
        hold on
    end

    grid on; grid minor;

    xlabel(plot_opts.x_label, 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel(plot_opts.y_label, 'Interpreter', 'latex', 'FontSize', font_size);

    % maxVal = max(rec1.Yaw_ref(obs_idx1)); minVal = min(rec1.Yaw_ref(obs_idx1)); 
    % len = maxVal-minVal; ratio = .1;
    % ylim([minVal-len*ratio maxVal+len*ratio]);
    % xlim([0 T])

    ax = gca;
    ax.FontSize = font_size; 
    ax.FontName = 'Times New Roman';
end