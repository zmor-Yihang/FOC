function pwm = svpwm_minmax(v_alpha, v_beta, U_DC, t, f_pwm)
% SVPWM调制函数 (min-max零序注入法) - 输出PWM波形
% 输入:
%   v_alpha - α轴电压 (V)
%   v_beta  - β轴电压 (V)
%   U_DC    - 直流母线电压 (V)
%   t       - 当前时间 (s)
%   f_pwm   - PWM载波频率 (Hz)，默认10kHz
% 输出:
%   pwm - 六路PWM波形 [pwm_ah; pwm_al; pwm_bh; pwm_bl; pwm_ch; pwm_cl]
%         取值 0 或 1

    % 默认PWM频率
    if nargin < 5
        f_pwm = 10000;  % 10kHz
    end

    % 反Clark变换: αβ -> abc
    u_a = v_alpha;
    u_b = -0.5 * v_alpha + 0.866025403784439 * v_beta;
    u_c = -0.5 * v_alpha - 0.866025403784439 * v_beta;
    
    % 归一化到 [-1, 1]
    inv_half_udc = 1.0 / (U_DC * 0.5);
    u_a = u_a * inv_half_udc;
    u_b = u_b * inv_half_udc;
    u_c = u_c * inv_half_udc;
    
    % 找最大最小值
    if u_a > u_b
        u_max = max(u_a, u_c);
        u_min = min(u_b, u_c);
    else
        u_max = max(u_b, u_c);
        u_min = min(u_a, u_c);
    end
    
    % 计算零序分量并注入
    u_zero = -0.5 * (u_max + u_min);
    
    % 映射到占空比 [0, 1]
    duty_a = (u_a + u_zero) * 0.5 + 0.5;
    duty_b = (u_b + u_zero) * 0.5 + 0.5;
    duty_c = (u_c + u_zero) * 0.5 + 0.5;
    
    % 占空比限幅
    duty_a = max(0.0, min(1.0, duty_a));
    duty_b = max(0.0, min(1.0, duty_b));
    duty_c = max(0.0, min(1.0, duty_c));
    
    % 生成PWM调制波形
    T_carrier = 1.0 / f_pwm;
    
    % 生成三角载波（中心对齐）
    t_mod = mod(t, T_carrier) / T_carrier;
    if t_mod < 0.5
        carrier = 2.0 * t_mod;       % 上升沿 0->1
    else
        carrier = 2.0 * (1.0 - t_mod); % 下降沿 1->0
    end
    
    % PWM比较生成波形
    pwm_ah = double(duty_a > carrier);
    pwm_bh = double(duty_b > carrier);
    pwm_ch = double(duty_c > carrier);
    
    % 下桥臂互补
    pwm_al = 1.0 - pwm_ah;
    pwm_bl = 1.0 - pwm_bh;
    pwm_cl = 1.0 - pwm_ch;
    
    % 输出六路PWM波形
    pwm = zeros(6, 1);
    pwm(1) = pwm_ah;  % A相上桥臂
    pwm(2) = pwm_al;  % A相下桥臂
    pwm(3) = pwm_bh;  % B相上桥臂
    pwm(4) = pwm_bl;  % B相下桥臂
    pwm(5) = pwm_ch;  % C相上桥臂
    pwm(6) = pwm_cl;  % C相下桥臂
    
end
