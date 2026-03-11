function id_ref = flux_weakening(v_d, v_q)
% FLUX_WEAKENING 弱磁控制器 (MATLAB Function Block)
%
% 输入:
%   v_d                 - D轴电压
%   v_q                 - Q轴电压
%
% 输出:
%   id_ref              - D轴参考电流 (<= 0)

% ---- 内部参数 ----
u_dc                 = 24.0;   % 母线电压 (V)
u_ref_ratio          = 0.95;   % 弱磁起始电压占比
ki                   = 0.1;    % 积分系数
id_min               = -5.0;   % 允许的最大负电流 (A), 输出下限
voltage_filter_const = 0.02;   % 电压低通滤波系数 (0.0~1.0)
% ------------------

persistent integral out_prev u_filtered;

% 初始化持久变量
if isempty(integral)
    integral   = 0.0;
    out_prev   = 0.0;
    u_filtered = 0.0;
end

% 输出范围: [id_min, 0]
out_min      = id_min;
out_max      = 0.0;
integral_max = abs(id_min);

%% 计算当前电压模值
u_mag = sqrt(v_d * v_d + v_q * v_q);

%% 低通滤波
u_filtered = u_filtered * (1.0 - voltage_filter_const) + u_mag * voltage_filter_const;

%% 目标电压参考值
u_ref = u_dc * u_ref_ratio;

%% 纯积分 PI 控制 (Kp = 0)
error = u_ref - u_filtered;

integral_increment = ki * error;

% 抗积分饱和：输出饱和时阻止同向积分
saturated_high = (out_prev >= out_max) && (integral_increment > 0);
saturated_low  = (out_prev <= out_min) && (integral_increment < 0);

if ~saturated_high && ~saturated_low
    integral = integral + integral_increment;
end

% 积分限幅
integral = max(-integral_max, min(integral_max, integral));

% PI输出 (Kp=0，仅积分项)
out = integral;

% 输出限幅
out = max(out_min, min(out_max, out));

out_prev = out;
id_ref   = out;

end
