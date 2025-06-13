%%
%input : 6*111001 
%output : 4*111001
%

%%
% === Step 2: Network Parameters 추출 ===
V_hat = net.IW{1};       % [n_hidden × n_input]
W_hat = net.LW{2,1};     % [n_output × n_hidden]
b1    = net.b{1};        % [n_hidden × 1]
b2    = net.b{2};        % [n_output × 1]

% === Step 3: NN 직접 계산===
n_sample = size(input, 2);
n_output = size(W_hat, 1);
h_hat = zeros(n_output, n_sample);

for k = 1:n_sample
    xn = input(:,k); 
    % h_hat(:,k) = W_hat * tansig(V_hat * z + b1) + b2;
    h_hat(:,k) = W_hat * tansig(V_hat * xn);
end

% === Plot 결과 비교 ===
titles = {'$h_1$', '$h_2$', '$h_3$', '$h_4$'};
figure('Position', [100 100 1000 600]);
for i = 1:n_output
    subplot(n_output,1,i)
    plot(time, output(i,:), 'k--', 'LineWidth', 1.2); hold on;
    plot(time, h_hat(i,:), 'b', 'LineWidth', 1.0);
    ylabel(titles{i}, 'Interpreter', 'latex');
    if i == 1
        legend('True $h$', 'NN $\hat{h}$', 'Interpreter','latex');
    end
    grid on;
end
xlabel('Time (s)')
sgtitle('NN vs True h', 'FontWeight', 'bold');