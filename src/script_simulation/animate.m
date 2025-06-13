clear
% figure(1); close
figure(1);

AINMATION_SAVE_FLAG = 0;
video_name = "sample";
gray = "#808080";

more_blue = "#0072BD";
more_red = "#A2142F";

line_width = 1.5;
font_size = 12;
T = 10;
accel = 100;

shadow_dt = .75;

%% 
rec1_name = "30-Apr-2025_20-46-35";
rec2_name = "30-Apr-2025_20-44-54";

%%
rec1 = load("results/" + rec1_name + ".mat");
rec2 = load("results/" + rec2_name + ".mat");
model = rec1.model;
Poses = rec1.Poses;
Poses2 = rec2.Poses;
assert(all(all(abs(Poses - Poses2) < 1e-5)), "The reference path is not the same");

rec1 = rec1.rec;   
rec2 = rec2.rec;

c1 = "blue"; c2="cyan";

%%
dt = rec1.X.Time(2) - rec1.X.Time(1);

%%
w = model.w;
lf = model.lf;
lr = model.lr;

%% SAVE VIDEO
if AINMATION_SAVE_FLAG
    v = VideoWriter("sim_result/"+video_name, 'MPEG-4');
    % v.Quality = 100;
    v.FrameRate = 1/dt/accel; 
    open(v);
end

%% ANIMATE
% reference path


% plot per time step
for t = 0:dt*accel:T
    t_idx1 = find(rec1.X.Time >= t, 1);
    t_idx2 = find(rec2.X.Time >= t, 1);

    if isempty(t_idx1) || isempty(t_idx2)
        break
    end

    X1 = rec1.X.Data(t_idx1); Y1 = rec1.Y.Data(t_idx1); Psi1 = rec1.Yaw.Data(t_idx1);
    X2 = rec2.X.Data(t_idx2); Y2 = rec2.Y.Data(t_idx2); Psi2 = rec2.Yaw.Data(t_idx2);
    
    ctrl_delta_f1 = rec1.control_delta_f.Data(t_idx1); ctrl_delta_r1 = rec1.control_delta_r.Data(t_idx1);
    ctrl_delta_f2 = rec2.control_delta_f.Data(t_idx2); ctrl_delta_r2 = rec2.control_delta_r.Data(t_idx2);
    delta_f1 = rec1.delta_f.Data(t_idx1); delta_r1 = rec1.delta_r.Data(t_idx1);
    delta_f2 = rec2.delta_f.Data(t_idx2); delta_r2 = rec2.delta_r.Data(t_idx2);

    cur_info_1.X = X1; cur_info_1.Y = Y1; cur_info_1.Psi = Psi1;
    cur_info_1.delta_f = delta_f1; cur_info_1.delta_r = delta_r1;
    cur_info_1.ctrl_delta_f = ctrl_delta_f1; cur_info_1.ctrl_delta_r = ctrl_delta_r1;
    cur_info_2.X = X2; cur_info_2.Y = Y2; cur_info_2.Psi = Psi2;
    cur_info_2.delta_f = delta_f2; cur_info_2.delta_r = delta_r2;
    cur_info_2.ctrl_delta_f = ctrl_delta_f2; cur_info_2.ctrl_delta_r = ctrl_delta_r2;

    plot_vhcl(cur_info_2, w,lf,lr, c2);
    plot_vhcl(cur_info_1, w,lf,lr, c1);

    % shadow plot
    shadow_num = floor(t/shadow_dt);
    for shadow_idx = 1:1:shadow_num
        t_idx1_shd = find(rec1.X.Time >= shadow_idx*shadow_dt, 1);
        t_idx2_shd = find(rec2.X.Time >= shadow_idx*shadow_dt, 1);

        cur_info_shd_1.X = rec1.X.Data(t_idx1_shd); cur_info_shd_1.Y = rec1.Y.Data(t_idx1_shd); cur_info_shd_1.Psi = rec1.Yaw.Data(t_idx1_shd);
        cur_info_shd_1.delta_f = rec1.delta_f.Data(t_idx1_shd); cur_info_shd_1.delta_r = rec1.delta_r.Data(t_idx1_shd);
        cur_info_shd_1.ctrl_delta_f = rec1.control_delta_f.Data(t_idx1_shd); cur_info_shd_1.ctrl_delta_r = rec1.control_delta_r.Data(t_idx1_shd);
        cur_info_shd_2.X = rec2.X.Data(t_idx2_shd); cur_info_shd_2.Y = rec2.Y.Data(t_idx2_shd); cur_info_shd_2.Psi = rec2.Yaw.Data(t_idx2_shd);
        cur_info_shd_2.delta_f = rec2.delta_f.Data(t_idx2_shd); cur_info_shd_2.delta_r = rec2.delta_r.Data(t_idx2_shd);
        cur_info_shd_2.ctrl_delta_f = rec2.control_delta_f.Data(t_idx2_shd); cur_info_shd_2.ctrl_delta_r = rec2.control_delta_r.Data(t_idx2_shd);

        if isempty(t_idx1_shd) || isempty(t_idx2_shd)
            break
        end

        plot_vhcl(cur_info_shd_2, w,lf,lr, c2);
        plot_vhcl(cur_info_shd_1, w,lf,lr, c1);
    end

    % shadow line
    plot(rec2.X.Data(1:t_idx2), rec2.Y.Data(1:t_idx2), "color", c2, "LineWidth", line_width, "LineStyle", "-"); hold on
    plot(rec1.X.Data(1:t_idx1), rec1.Y.Data(1:t_idx1), "color", c1, "LineWidth", line_width, "LineStyle", "-"); hold on

    %  time
    text(.25,-15,0, ...
        sprintf( ...
            'Time: %.2f/%.0f s (%.1f%%) ', t, T, round(t/T*100, 3) ...
        ), ...
        "FontSize", font_size, ...
        "FontName", "Times New Roman" ...
        );

    % set(gca, 'XTickLabel', [])
    % set(gca, 'YTickLabel', [])
    set(gca, 'FontSize', font_size, 'FontName', 'Times New Roman')

    plot(Poses(:,1), Poses(:,2), 'r', 'LineWidth', line_width, 'LineStyle', '--'); hold on
    xlim([0 50])
    ylim([-10 11]);
    daspect([1 .9 1])
    grid on

    % get frame
    drawnow

    if AINMATION_SAVE_FLAG
        f = getframe(gcf);
        writeVideo(v, f);
    end

    pause(0.001)
    clf
end

if AINMATION_SAVE_FLAG
    close(v);
end


function plot_vhcl(cur_info, w,lf,lr, color)
    X = cur_info.X; Y = cur_info.Y; Psi = cur_info.Psi;
    delta_f = cur_info.delta_f; delta_r = cur_info.delta_r;
    ctrl_delta_f = cur_info.ctrl_delta_f; ctrl_delta_r = cur_info.ctrl_delta_r;

    w = 1;
    line_width = 1.5;

    fr = [lf, -w/2]; fl = [lf, w/2];
    rr = [-lr, -w/2]; rl = [-lr, w/2];

    global_fr = [X; Y] + rotMat(Psi) * fr';
    global_fl = [X; Y] + rotMat(Psi) * fl';
    global_rr = [X; Y] + rotMat(Psi) * rr';
    global_rl = [X; Y] + rotMat(Psi) * rl';

    global_X_edge = [global_fl(1), global_fr(1), global_rr(1), global_rl(1), global_fl(1)];
    global_Y_edge = [global_fl(2), global_fr(2), global_rr(2), global_rl(2), global_fl(2)];

    patch("XData", global_X_edge, "YData", global_Y_edge, ...
        "FaceColor", color, ...
        "EdgeColor", color, ...
        "LineWidth", line_width, ...
        "FaceAlpha", 0.2, ...
        "EdgeAlpha", 0.5 ...
    ); hold on

    steer_arrow = 1.5;

    % front steer
    front_p1 = [X; Y] + rotMat(Psi) * [lf, 0]';
    front_p2 = front_p1 + rotMat(Psi+delta_f) * [steer_arrow, 0]';
    front_p3 = front_p1 + rotMat(Psi+ctrl_delta_f) * [steer_arrow, 0]';
    
    dp12 = front_p2 - front_p1;
    dp13 = front_p3 - front_p1;
    quiver(front_p1(1),front_p1(2),dp12(1),dp12(2),0, ...
        "Color", color, "LineWidth", line_width, "MaxHeadSize", 2, ...
        "AutoScale", "off", "ShowArrowHead", "on" ...
    ); hold on
    quiver(front_p1(1),front_p1(2),dp13(1),dp13(2),0, ...
        "Color", color, "LineWidth", line_width, "MaxHeadSize", 2, ...
        "AutoScale", "off", "ShowArrowHead", "on" ...
    ); hold on
    grid

    % rear steer
    rear_p1 = [X; Y] + rotMat(Psi) * [-lr, 0]';
    rear_p2 = rear_p1 + rotMat(Psi+delta_r) * [steer_arrow, 0]';
    rear_p3 = rear_p1 + rotMat(Psi+ctrl_delta_r) * [steer_arrow, 0]';
    
    dp12_rear = rear_p2 - rear_p1;
    dp13_rear = rear_p3 - rear_p1;
    quiver(rear_p1(1),rear_p1(2),dp12_rear(1),dp12_rear(2),0, ...
        "Color", color, "LineWidth", line_width, "MaxHeadSize", 2, ...
        "AutoScale", "off", "ShowArrowHead", "on" ...
    ); hold on
    quiver(rear_p1(1),rear_p1(2),dp13_rear(1),dp13_rear(2),0, ...
        "Color", color, "LineWidth", line_width, "MaxHeadSize", 2, ...
        "AutoScale", "off", "ShowArrowHead", "on" ...
    ); hold on
    
end

function rotmat = rotMat(theta)
    rotmat = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end

%% Backup
%{
% tracking 1
    axes('Position',[.2 .7 .25 .2])
    box on
    shadow = max(1, t_idx1-1e2:1:t_idx1);
    shadow = shadow';

    plot(data4.q1.Time(shadow), data4.q1.Data(shadow), "color", c4, "LineWidth", line_width, "LineStyle", "-"); hold on
    plot(data3.q1.Time(shadow), data3.q1.Data(shadow), "color", c3, "LineWidth", line_width, "LineStyle", "-"); hold on
    plot(data2.q1.Time(shadow), data2.q1.Data(shadow), "color", c2, "LineWidth", line_width, "LineStyle", "-"); hold on
    plot(data1.q1.Time(shadow), data1.q1.Data(shadow), "color", c1, "LineWidth", line_width, "LineStyle", "-"); hold on

    plot(data1.r1.Time(shadow), data1.r1.Data(shadow), "color", 'red', "LineWidth", line_width, "LineStyle", "--"); hold on

    % xlabel("Time / s", "Interpreter", "latex")
    ylabel("$q_1$ / rad", "Interpreter", "latex")
    grid on
    set(gca, 'XTickLabel', [])
    set(gca, 'FontSize', 12, 'FontName', 'Times New Roman')

    maxVal = max(data1.r1.Data); minVal = min(data1.r1.Data); 
    len = maxVal-minVal; ratio = .3;
    ylim([minVal-len*ratio maxVal+len*ratio]);
    tmp_t = data1.r1.Time(shadow);
    if tmp_t(end) ~= 0
        xlim([tmp_t(1) tmp_t(end)])
    end

%}