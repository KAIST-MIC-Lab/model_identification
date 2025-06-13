clear

addpath(genpath("models"))
addpath("ref_path")
addpath("controller")

RESULT_SAVE_FLAG = 0;   % save the result as a .mat file in the results folder
FIGURE_PLOT_FLAG = 1;   % plot the result
FIGURE_SAVE_FLAG = 0;   % save the figure as .png and .eps

%% RECORER
rec.Name = [ ...
    "ref1",              "ref(1)";
    "ref2",              "ref(2)";
    "ref_dot1",          "ref_dot(1)";
    "ref_dot2",          "ref_dot(2)";

    "x_1",                "x(1)";
    "x_2",                "x(2)";
    "x_3",                "x(3)";
    "x_4",                "x(4)";

    "x_hat1",           "x_hat(1)";
    "x_hat2",           "x_hat(2)";
    "x_hat3",           "x_hat(3)";
    "x_hat4",           "x_hat(4)";

    "g_hat1",          "g_hat(1)";
    "g_hat2",          "g_hat(2)";
    "g_hat3",          "g_hat(3)";
    "g_hat4",          "g_hat(4)";

    "g1",              "g(1)";
    "g2",              "g(2)";
    "g3",              "g(3)";
    "g4",              "g(4)";

    "u1",              "u(1)";
    "u2",              "u(2)";

    "F_hat1",          "F_hat(1)";
    "F_hat2",          "F_hat(2)";

    "F1",              "F(1)";
    "F2",              "F(2)";

    "W_norm",          "W_norm";
    "V_norm",          "V_norm";

    ];
for r_idx = 1:1:size((rec.Name),1)
    rec.(rec.Name(r_idx,1)).Data = [];
    rec.(rec.Name(r_idx,1)).Time = [];
end

%% SIMULATION SETTING
T = 5;                 % simulation time
ctrl_dt = 1e-3;         % controller sampling time
dt = 1e-3;       % simulation sampling time
rpt_dt = 1;             % report time (on console)
t = 0:dt:T;             % time vector
num_t = length(t);     % number of time step
% 제어기 ctrl_dt 안 쓰고 있음.
last_ctrl_time = 0;    

%% REPORT SETTING
fprintf("\n")
fprintf("      *** SIMULATION INFORMATION ***\n")
fprintf("Termiation Time  : %.2f\n", T)
fprintf("Controller dt    : %.2e\n", ctrl_dt)
fprintf("Simulation dt    : %.2e\n", dt)
fprintf("Report dt        : %.2e\n", rpt_dt)
fprintf("\n")
fprintf("RESULT_SAVE_FLAG : %d\n", RESULT_SAVE_FLAG)
fprintf("FIGURE_PLOT_FLAG : %d\n", FIGURE_PLOT_FLAG)
fprintf("FIGURE_SAVE_FLAG : %d\n", FIGURE_SAVE_FLAG)
fprintf("\n")

%% SYSTEM AND REFERENCE DEFINITION
robot = initStructure();       

%% MODEL LOAD (parameter setting)
model = loadOpts("simple_model.m", dt);

%intialize ref
ref = robot.q;
ref_dot = robot.q_dot; 
ref_ddot = zeros(2,1);

ri = robot.q;
rdoti = robot.q_dot;
rf = [pi/2; pi/2]; % final position
rdotf = [0; 0]; % final velocity

%intialize system
u = zeros(2,1);
x = [robot.q; robot.q_dot];
x_hat = [robot.q; robot.q_dot];
F_hat = zeros(2,1);

%% identification LOAD

eta1 = 1e3;
eta2 = 1e0;
rho1 = 0.0;
rho2 = 0.0;
n_i = 6;
n_h = 8;
n_o = 4;

A = -20 * eye(4);
W0 = 1e-1 * randn(n_o,n_h) + 0;
V0 = 1e-1 * randn(n_h,n_i) + 0;

W_hat = W0;
V_hat = V0;

%% Control gain
Kp = diag([100 100]);
Kd = diag([20 20]);

%% MAIN LOOP
fprintf("SIMULATION RUNNING...\n")

for t_idx = 1:1:num_t
    cur_t = t(t_idx);
    
    x = [robot.q; robot.q_dot];
    [ref, ref_dot, ref_ddot] = ref_gen(ri, rdoti, rf, rdotf, cur_t, T/2, 2, 2); 
    
    % control input
    [M, C, G, F] = two_link_model (robot.q, robot.q_dot, model);
    e = ref - robot.q;
    edot = ref_dot - robot.q_dot;

    u = M * (ref_ddot + Kp * e + Kd * edot) + C * robot.q_dot + G;

    f = [robot.q_dot
         M \ (- C * robot.q_dot - G + u)];
    
    Aug_F = [zeros(2,1); - inv(M) * F];

    % identification
    x_hat_bar = [x_hat; u];
    x_tilde = x - x_hat;

    g_hat = NN(W_hat, V_hat, x_hat_bar);
    g = f + Aug_F - A * x; 
    
    % identifier
    x_hat_dot = A * x_hat + g_hat;

    W_norm = norm(W_hat,'fro');
    V_norm = norm(V_hat, 'fro');


    % calculate the friction force
    Aug_F_hat = g_hat + A * x_hat - f;
    F_hat = - M * Aug_F_hat(3:4);

    % record data
    for r_idx = 1:1:length(rec.Name)
        rec.(rec.Name(r_idx,1)).Data = [rec.(rec.Name(r_idx,1)).Data; eval(rec.Name(r_idx,2))];
        rec.(rec.Name(r_idx,1)).Time = [rec.(rec.Name(r_idx,1)).Time; cur_t];
    end


    % update the identifier
    x_hat = x_hat + x_hat_dot * dt;

    % update the weights
    W_hat_dot = - eta1 * (x_tilde' * inv(A))' * activate(V_hat * x_hat_bar)' - rho1 * norm(x_tilde) * W_hat;
    Lambda = diag(activate(V_hat * x_hat_bar).^2);
    V_hat_dot = - eta2 * (x_tilde' * inv(A) * W_hat * (eye(n_h) - Lambda))' * x_hat_bar' - rho2 * norm(x_tilde) * V_hat;
    
    W_hat = W_hat + W_hat_dot * dt;
    V_hat = V_hat + V_hat_dot * dt;



    % Simulation Step forward
    robot = step(robot, u, model);

    % Report
    if mod(t_idx, rpt_dt/dt) == 0
        fprintf('Simulation Time: %.2f\n', cur_t)
    end

end

fprintf("SIMULATION is Terminated\n")

%% RESULT REPORT AND SAVE
whatTimeIsIt = string(datetime('now','Format','d-MMM-y_HH-mm-ss'));

if RESULT_SAVE_FLAG
    fprintf("\n")
    fprintf("RESULT SAVING...\n")

    saveName = "results/"+whatTimeIsIt+".mat";
    save(saveName, 'rec', 'model')

    fprintf("RESULT is Saved as \n \t%s\n", saveName)
end

if FIGURE_PLOT_FLAG
    fprintf("\n")
    fprintf("FIGURE PLOTTING...\n")

    plotter(rec, model)
    fprintf("FIGURE PLOTTING is Done\n")

    if FIGURE_SAVE_FLAG
        fprintf("\n")
        fprintf("FIGURE SAVING...\n")
        
        saveName = "figures/"+whatTimeIsIt;
        [~,~] = mkdir(saveName);

        for idx = 1:1:4   
            f_name = saveName + "/Fig" + string(idx);
    
            saveas(figure(idx), f_name + ".png")
            exportgraphics(figure(idx), f_name+'.eps')
        end

        fprintf("FIGURE is Saved in \n \t%s\n", saveName)
    end
end

beep()


function g_hat = NN(W_hat, V_hat, x_hat_bar)
    output = W_hat * activate(V_hat * x_hat_bar);
    g_hat = output;

end

function [V_new,W_new] = learningrule(V_old, W_old, e, x_hat_bar, A, eta1, eta2, rho1, rho2, n_h)

    W_hat = W_old;
    V_hat = V_old;
    
    % identification
    x_tilde = e;
    
    Lambda = zeros(n_h,n_h);
    Lambda = diag(tanh(V_hat * x_hat_bar).^2);
    
    % update the weights
    W_hat_dot = - eta1 * (x_tilde' * inv(A))' * activate(V_hat*x_hat_bar)' - rho1 * norm(x_tilde) * W_hat;
    V_hat_dot = - eta2 * (x_tilde' * inv(A) * W_hat * (eye(size(Lambda)) - Lambda))' * x_hat_bar' - rho2 * norm(x_tilde) * V_hat;
    
    W_new = W_hat_dot;
    V_new = V_hat_dot;

end

function sigma = activate(x)
    sigma = tanh(x) + 2*ones(size(x));
end