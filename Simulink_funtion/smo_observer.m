function [e_alpha, e_beta] = smo_observer(i_alpha, i_beta, u_alpha, u_beta)
% 滑模观测器 (SMO) - MATLAB Function Block
% 基于饱和函数的滑模电流观测器，输出估算反电动势
% 角度与速度估算由独立的 pll_estimator MATLAB Function Block 完成
%
% 输入:
%   i_alpha  - 实测 α 轴电流 (A)
%   i_beta   - 实测 β 轴电流 (A)
%   u_alpha  - α 轴电压参考 (V)
%   u_beta   - β 轴电压参考 (V)
%
% 输出:
%   e_alpha  - 估算反电动势 α 轴分量（接 pll_estimator 输入）
%   e_beta   - 估算反电动势 β 轴分量（接 pll_estimator 输入）

%% ============ 参数配置区域 ============
rs          = 2.0;       % 定子电阻 (Ω)
ls          = 0.003;     % 定子电感 (H)
poles       = 4;         % 极对数
ts          = 1e-4;      % 控制周期 (s)，与 PWM 中断周期一致
k_slide     = 5.0;       % 滑模增益
k_lpf       = 0.3;       % 反电动势低通滤波系数 (0~1，越大截止频率越高)
boundary    = 0.5;       % 滑模边界层厚度
%% =====================================

%% 内部持久状态
persistent i_alpha_est i_beta_est
persistent e_alpha_s e_beta_s
persistent z_alpha z_beta

if isempty(i_alpha_est)
    i_alpha_est = 0;
    i_beta_est  = 0;
    e_alpha_s   = 0;
    e_beta_s    = 0;
    z_alpha     = 0;
    z_beta      = 0;
end

%% 离散状态方程系数
F = 1.0 - rs * ts / ls;
G = ts / ls;

%% 1. 更新电流估算
i_alpha_est = F * i_alpha_est + G * (u_alpha - e_alpha_s - z_alpha);
i_beta_est  = F * i_beta_est  + G * (u_beta  - e_beta_s  - z_beta);

%% 2. 计算电流误差
i_err_alpha = i_alpha_est - i_alpha;
i_err_beta  = i_beta_est  - i_beta;

%% 3. 滑模控制量（饱和函数）
z_alpha = k_slide * sat_fun(i_err_alpha, boundary);
z_beta  = k_slide * sat_fun(i_err_beta,  boundary);

%% 4. 低通滤波提取反电动势
e_alpha_s = (1.0 - k_lpf) * e_alpha_s + k_lpf * z_alpha;
e_beta_s  = (1.0 - k_lpf) * e_beta_s  + k_lpf * z_beta;

%% 5. 输出反电动势（由外部 pll_estimator Block 完成角速度/角度估算）
e_alpha = e_alpha_s;
e_beta  = e_beta_s;

end % function smo_observer


%% ---- 局部辅助函数：饱和函数 ----
function y = sat_fun(x, delta)
% 边界层内线性过渡，边界层外饱和
if x > delta
    y = 1.0;
elseif x < -delta
    y = -1.0;
else
    y = x / delta;
end
end
