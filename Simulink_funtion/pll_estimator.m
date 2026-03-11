function [theta_est, speed_rpm, speed_rad_s] = pll_estimator(e_alpha, e_beta)
% PLL 锁相环 - MATLAB Function Block
% 基于估算反电动势估算转子角度与速度
% 作为独立的 Simulink MATLAB Function Block 使用，参数在内部配置区设置
%
% 输入:
%   e_alpha     - 估算反电动势 α 轴分量（来自 smo_observer 或 luenberger_observer 的信号线）
%   e_beta      - 估算反电动势 β 轴分量
%
% 输出:
%   theta_est   - 估算电角度 (rad)，归一化到 (-π, π]
%   speed_rpm   - 滤波后的估算转速 (RPM)
%   speed_rad_s - 估算电角速度 (rad/s)，反馈给 luenberger_observer 的 we 输入（经 Unit Delay）

%% ============ 参数配置区域 ============
poles       = 4;        % 极对数
ts          = 1e-4;     % 控制周期 (s)，与 PWM 中断周期一致
pll_fc      = 100.0;    % PLL 截止频率 (Hz)
k_speed_lpf = 0.1;      % 速度低通滤波系数 (0~1)
%% =====================================

%% 内部持久状态
persistent theta
persistent speed_filt
persistent pll_integ
persistent pll_out_state

if isempty(theta)
    theta         = 0;
    speed_filt    = 0;
    pll_integ     = 0;
    pll_out_state = 0;
end

%% PLL PI 参数计算
wn              = 2.0 * pi * pll_fc;
kp_pll          = 2.0 * wn;               % zeta = 1
ki_pll          = wn * wn * ts;           % 预乘 ts，与 C 代码一致
max_rpm         = 10000.0;
max_speed_rad_s = max_rpm * 2.0 * pi * poles / 60.0;

%% PLL 误差：ΔE = -(Êα·cosθ̂ + Êβ·sinθ̂)
pll_err = -(e_alpha * cos(theta) + e_beta * sin(theta));

%% PI 控制（带抗积分饱和）
proportional       = kp_pll * pll_err;
integral_increment = ki_pll * pll_err;

saturated_high = (pll_out_state >= max_speed_rad_s)  && (integral_increment > 0);
saturated_low  = (pll_out_state <= -max_speed_rad_s) && (integral_increment < 0);
if ~saturated_high && ~saturated_low
    pll_integ = pll_integ + integral_increment;
end
pll_integ     = max(-max_speed_rad_s, min(max_speed_rad_s, pll_integ));
pll_out_state = proportional + pll_integ;
pll_out_state = max(-max_speed_rad_s, min(max_speed_rad_s, pll_out_state));

speed_rad_s = pll_out_state;

%% 机械转速 (RPM) 并低通滤波
speed_rpm_raw = speed_rad_s * 60.0 / (2.0 * pi * poles);
speed_filt    = (1.0 - k_speed_lpf) * speed_filt + k_speed_lpf * speed_rpm_raw;
speed_rpm     = speed_filt;

%% 角度积分并归一化到 (-π, π]
theta = theta + speed_rad_s * ts;
if theta > pi
    theta = theta - 2.0 * pi;
elseif theta <= -pi
    theta = theta + 2.0 * pi;
end
theta_est = theta;

end % function pll_estimator
