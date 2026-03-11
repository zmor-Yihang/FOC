function [e_alpha, e_beta] = luenberger_observer(i_alpha, i_beta, u_alpha, u_beta, we)
% 龙伯格观测器 (Luenberger Observer) - MATLAB Function Block
% 基于全阶状态观测器估算反电动势，角度与速度五由独立的 pll_estimator Block 完成
%
% 输入:
%   i_alpha  - 实测 α 轴电流 (A)
%   i_beta   - 实测 β 轴电流 (A)
%   u_alpha  - α 轴电压参考 (V)
%   u_beta   - β 轴电压参考 (V)
%   we       - 上一拍估算电角速度 (rad/s)，来自 pll_estimator 的 speed_rad_s
%              输出经由 Unit Delay 反馈以避免代数环
%
% 输出:
%   e_alpha  - 估算反电动势 α 轴分量（接 pll_estimator 输入）
%   e_beta   - 估算反电动势 β 轴分量（接 pll_estimator 输入）

%% ============ 参数配置区域 ============
rs          = 0.11;        % 定子电阻 (Ω)
ls          = 0.00097;     % 定子电感 (H)
poles       = 4;           % 极对数
ts          = 1e-4;        % 控制周期 (s)，与 PWM 中断周期一致
l1          = -17396.0;    % 电流观测器增益（负值保证稳定）
l2          = 74350.0;     % 反电动势观测器增益
%% =====================================

%% 内部持久状态
persistent i_alpha_est i_beta_est
persistent e_alpha_est e_beta_est

if isempty(i_alpha_est)
    i_alpha_est = 0;
    i_beta_est  = 0;
    e_alpha_est = 0;
    e_beta_est  = 0;
end

%% 离散状态方程系数
k_tl  = ts / ls;          % Ts / Ls
k_trl = ts * rs / ls;     % Ts * Rs / Ls

%% 1. 计算电流估算误差: i_est(k) - i_meas(k)
i_err_alpha = i_alpha_est - i_alpha;
i_err_beta  = i_beta_est  - i_beta;

%% 2. 电流观测器更新 (使用上一拍估算电角速度 we)

i_alpha_est = i_alpha_est - k_trl * i_alpha_est ...
              - k_tl * e_alpha_est + k_tl * u_alpha ...
              + l1 * ts * i_err_alpha;

i_beta_est  = i_beta_est  - k_trl * i_beta_est ...
              - k_tl * e_beta_est  + k_tl * u_beta ...
              + l1 * ts * i_err_beta;

%% 3. 反电势观测器更新 (含交叉耦合项)
e_alpha_new = e_alpha_est - we * ts * e_beta_est  + l2 * ts * i_err_alpha;
e_beta_new  = e_beta_est  + we * ts * e_alpha_est + l2 * ts * i_err_beta;

e_alpha_est = e_alpha_new;
e_beta_est  = e_beta_new;

%% 4. 输出反电动势（由外部 pll_estimator Block 完成角速度/角度估算）
e_alpha = e_alpha_est;
e_beta  = e_beta_est;

end % function luenberger_observer
