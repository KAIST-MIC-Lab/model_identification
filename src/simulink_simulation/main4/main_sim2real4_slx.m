
%% FASTEN YOUR SEATBELT
clear

load('sim_results/logsout_org.mat');  % logsout이 포함된 .mat 파일

% 예시: logsout으로부터 원하는 데이터를 TimeSeries 형태로 추출
X_ts = logsout_org.get('X').Values;       
u_ts = logsout_org.get('u').Values;
Ref_ts = logsout_org.get('Ref').Values;

% Time and position data
time = X_ts.Time;                     % Nx1
q_data = squeeze(X_ts.Data(:,1:2));  % Nx2 (1,2번째 상태: position)

%% Savitzky-Golay filter parameters
window_length = 51;  % must be odd
poly_order = 3;

% Apply Savitzky-Golay filter to differentiate
qdot_data = zeros(size(q_data));
for i = 1:2
    qdot_data(:,i) = sgolayfilt(q_data(:,i), poly_order, window_length, 1, [], dt);
end

%% Create timeseries (2×1 벡터 형태로 구성)
qdot_ts = timeseries(qdot_data, time);
qdot_ts.Name = 'qdot_ts';
qdot_ts.DataInfo.Units = 'rad/s';

%% 결과 확인
plot(qdot_ts.Time, qdot_ts.Data)
legend('q̇₁', 'q̇₂'); xlabel('Time [s]'); ylabel('Velocity [rad/s]');
title('qdot\_ts from Savitzky-Golay differentiation'); grid on;



RESULT_PLOT_FLAG =1;
RUN_FLAG = 1;           % run the simulink simulation
RESULT_SAVE_FLAG = 1;   % save the result as a .mat file in the results folder

slx_name = "main_real4.slx";  % simulink file name

Traj_flag = 2;  % ← 1 또는 2로 설정

% 파일 이름 생성
file_path = fullfile("raw_data", sprintf("%d.mat", Traj_flag));

% 파일 존재 여부 확인 후 로드
if exist(file_path, 'file')
    fprintf("Loading %s...\n", file_path);
    load(file_path);

    % Traj_flag == 2일 경우 마지막 열 제거
    if Traj_flag == 2
        if exist('data_2901', 'var') && size(data_2901, 2) > 1
            data_2901(:, end) = [];  % 마지막 열 제거
        end
        if exist('data_2902', 'var') && size(data_2902, 2) > 1
            data_2902(:, end) = [];  % 마지막 열 제거
        end
        if exist('time', 'var') && size(time, 2) > 1
            time(:, end) = [];       % 마지막 열 제거
        end
    end
else
    error("File %s not found.", file_path);
end



%% SIMULATION SETTING
ctrl_dt = 2e-3;         % controller sampling time
dt = 2e-3;       % simulation sampling time
T = (length(data_003) -1)* dt;
t = 0:dt:T;             % time vector


%% REPORT SETTING
fprintf("\n")
fprintf("      *** SIMULATION INFORMATION ***\n")
fprintf("Termiation Time  : %.2f\n", T)
fprintf("Controller dt    : %.2e\n", ctrl_dt)
fprintf("Simulation dt    : %.2e\n", dt)
fprintf("\n")

%% INITIAL CONDITION
X0 = [data_2901(1,1);data_2902(1,1);data_2901(2,1);data_2902(2,1)];
X_hat_0 = X0;

u = zeros(2,1);
F_hat = zeros(2,1);
friction = zeros(2,1);

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
   'b2', 0.7, ...
   'fc1', 0.1, ...
   'fc2', 0.1, ...
   'k1', 1500, ...
   'k2', 1500);


Kp = [100 0
      0   100];
Kd = [20 0
      0  20];

%% identification LOAD
eta1 = 1e3;
eta2 = 1e2; 
rho1 = 0.0;
rho2 = 0.0;
n_i = 6;
n_h = 100;
n_o = 4;
a = 20;
A = -a * eye(4);
rng(18);
W0 = 1e-2 * randn(n_o,n_h) + 0;
V0 = 1e-2 * randn(n_h,n_i) + 0;

% W = W_data(:,:,14000);
% V = V_data(:,:,14000);

%% MAIN SIMULATION RUN
if RUN_FLAG
    fprintf("SIMULINK SIMULATION is Running...\n")

    sim_result = sim(slx_name);
    
    fprintf("SIMULINK SIMULATION is Done!\n")

end

%% RESULT REPORT AND SAVE
whatTimeIsIt = string(datetime('now','Format','d-MMM-y_HH-mm-ss'));

if RESULT_PLOT_FLAG
    fprintf("\n")
    fprintf("RESULT PLOTTING...\n")

    logsout = sim_result.logsout;

    % === Plotting function 선택 ===
    if Traj_flag == 1
        main_plotter4_1(logsout);
    elseif Traj_flag == 2
        main_plotter4_2(logsout);
    else
        error("Invalid flag: must be 1 or 2");
    end

    if RESULT_SAVE_FLAG
        fprintf("RESULT SAVING...\n")

        % 결과 폴더 확인
        result_folder = "sim2real_results";
        if ~exist(result_folder, 'dir')
            mkdir(result_folder);
        end

        % 파일 베이스 이름 (flag 붙이기)
        file_base = sprintf("%d_n_h_%g_a_%g_eta1_%g_eta2_%g", Traj_flag, n_h, a, eta1, eta2);

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

% end