% === FOR LOOP BEGIN ===
% for a = 100
% for eta1 = 5e4
% for N = 10000


%% FASTEN YOUR SEATBELT
% clear
RESULT_PLOT_FLAG = 1;
RUN_FLAG = 1;           % run the simulink simulation
RESULT_SAVE_FLAG = 0;   % save the result as a .mat file in the results folder

Traj_flag = 1;  % ← 1 또는 2로 설정

slx_name = "main_sim5.slx";  % simulink file name

%% SIMULATION SETTING
ctrl_dt = 1e-3;         % controller sampling time
dt = 1e-3;       % simulation sampling time

    if Traj_flag == 1
        % === Trajectory 1 ===
        q0 = [-pi/2; 0];
        qd1 = [pi/4; pi/2];
        qd2 = [pi/4 + pi/2; -pi/2];
        % waypoints = [q0, ... 
        %      qd1, qd3, qd1, qd4, qd3, qd1, qd2, qd3, qd2, qd1, ...
        %      qd3, qd2, qd4, qd2, qd1, qd3, qd4, qd1, qd3, qd2, ...
        %      qd1, q0];
        waypoints = [q0, ... 
                     qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, ...
                     qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, ...
                     qd1, q0];
        traj_duration = 4.0;
        init_duration = 4.0;
        end_duration = 4.0;
    else
        % === Trajectory 2 ===
        q0 = [-pi/2; 0];
        qd1 = [-pi/3; 2*pi/3];
        qd2 = [pi/3; -2*pi/3];
        waypoints = [q0, ... 
                     qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, ...
                     qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, qd1, qd2, ...
                     qd1, q0];
        traj_duration = 20.0;
        init_duration = 4.0;
        end_duration = 4.0;
    end
    
    % === 공통 처리 ===
    num_segments = size(waypoints, 2) - 1;
    
    traj_times = traj_duration * ones(1, num_segments);
    traj_times(1) = init_duration;
    traj_times(end) = end_duration;
    
    Tidle = 0;

    % Compute total time
    Ttotal = Tidle * (num_segments + 1) + sum(traj_times);

T = Ttotal;                 % simulation time

t = 0:dt:T;             % time vector

%% REPORT SETTING
fprintf("\n")
fprintf("      *** SIMULATION INFORMATION ***\n")
fprintf("Termiation Time  : %.2f\n", T)
fprintf("Controller dt    : %.2e\n", ctrl_dt)
fprintf("Simulation dt    : %.2e\n", dt)
fprintf("\n")

%% INITIAL CONDITION
X0 = [-pi/2; 0; 0; 0];
X_hat_0 = X0;

u = zeros(2,1);
F_hat = zeros(2,1);

%% Model Load


params = struct( ...
   'm1', 2.465, ...
   'm2', 2.465, ...
   'l1', 0.2, ...
   'l2', 0.2, ...
   'lc1', 0.13888, ...
   'lc2', 0.13888, ...
   'I1', 0.06911, ...
   'I2', 0.06911, ...
   'I1m', 9*9*1002e-7, ...
   'I2m', 9*9*1002e-7, ...
   'b1', 0.5, ...
   'b2', 0.5, ...
   'fc1', 0.1, ...
   'fc2', 0.1, ...
   'k1', 1500, ...
   'k2', 1500);


Kp = [100 0
      0   100];
Kd = [20 0
      0  20];

%% identification LOAD
eta1 = 1e-1;
eta2 = 1e2;
lambda = 1e0; %(decay rate lambda = 3/T, 1e3 - > 3ms)
rho1 = 0.00;
rho2 = 0.00;
% N = 5;
n_i = 6;
n_h = 20;
n_o = 4;

a = 40;
A = -a * eye(4);


rng(18);
W0 = 1e-1 * randn(n_o,n_h);
V0 = 1e-1 * randn(n_h,n_i);

% J_W_t_buffer = zeros(n_o * n_h, N);  % For W gradients
% J_V_t_buffer = zeros(n_h * n_i, N);  % For V gradients

% W = W_data(:,:,14000);
% V = V_data(:,:,14000);


% 파일 베이스 이름 (flag 붙이기)
file_base = sprintf("%d_n_h_%g_a_%g_eta1_%g_eta2_%g_rho1_%g_rho2_%g", Traj_flag, n_h, a, eta1, eta2, rho1, rho2);

%% MAIN SIMULATION RUN
if RUN_FLAG
    fprintf("SIMULINK SIMULATION is Running...\n")

    sim_result = sim(slx_name);
    
    fprintf("SIMULINK SIMULATION is Done!\n")
    logsout = sim_result.logsout;
    assignin('base', 'logsout', logsout); 
end

%% RESULT REPORT AND SAVE
whatTimeIsIt = string(datetime('now','Format','d-MMM-y_HH-mm-ss'));

if RESULT_PLOT_FLAG
    fprintf("\n")
    fprintf("RESULT PLOTTING...\n")

    logsout = sim_result.logsout;
    assignin('base', 'logsout', logsout); 
    % === Plotting function 선택 ===
    if Traj_flag == 1
        main_plotter5_1(logsout);
    elseif Traj_flag == 2
        main_plotter5_2(logsout);
    else
        error("Invalid flag: must be 1 or 2");
    end

    if RESULT_SAVE_FLAG
        fprintf("RESULT SAVING...\n")

        % 결과 폴더 확인
        result_folder = "sim_results";
        if ~exist(result_folder, 'dir')
            mkdir(result_folder);
        end
        % logsout 저장
        save_mat = fullfile(result_folder, file_base + ".mat");
        % save(save_mat, 'logsout');
        fprintf("  → logsout saved as %s\n", save_mat);


        % Figure 저장
        figs = findall(0, 'Type', 'figure');
        for i = 1:length(figs)
            if isvalid(figs(i)) && strcmp(get(figs(i), 'Type'), 'figure')
                fig_number = get(figs(i), 'Number');
                if ~isempty(fig_number) && isnumeric(fig_number)
                    save_png = fullfile(result_folder, sprintf("%s_figure%d.png", file_base, fig_number));
                    saveas(figs(i), save_png);
                    fprintf("  → Figure %d saved as %s.\n", fig_number, save_png);
                end
            end
        end
    end
end



% === FOR LOOP END ===
% end
% end
% end


