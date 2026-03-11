function out = pi_controller(error)
% PI控制器 - MATLAB Function
% 输入:
%   error    - 误差 (给定 - 反馈)
% 输出:
%   out      - 控制输出

%% ============ 参数配置区域 ============
kp = 1.0;       % 比例系数
ki = 0.1;       % 积分系数
out_min = -10;  % 输出下限
out_max = 10;   % 输出上限
%% =====================================

% 持久变量 - 保持积分状态和上次输出
persistent integral
persistent last_out

% 初始化持久变量
if isempty(integral)
    integral = 0;
    last_out = 0;
end

% 积分限幅上限
integral_max = max(abs(out_min), abs(out_max));

% 比例项
proportional = kp * error;

% 积分增量
integral_increment = ki * error;

% 抗积分饱和：输出饱和时，阻止同向积分
saturated_high = (last_out >= out_max) && (integral_increment > 0);
saturated_low  = (last_out <= out_min) && (integral_increment < 0);

if ~saturated_high && ~saturated_low
    integral = integral + integral_increment;
end

% 积分限幅
if integral > integral_max
    integral = integral_max;
elseif integral < -integral_max
    integral = -integral_max;
end

% PI输出 = 比例 + 积分
out = proportional + integral;

% 输出限幅
if out > out_max
    out = out_max;
elseif out < out_min
    out = out_min;
end

% 保存本次输出用于下次抗饱和判断
last_out = out;

end
