def calculate_st_gains(rs, ls, T, f):
    """
    计算ST公司的龙伯格观测器增益
    
    参数:
        rs: 定子电阻 (Ω)
        ls: 定子电感 (H)
        T: PWM周期 (s)
        f: 观测器极点放置系数 (5 <= f <= 10)
    
    返回:
        K1, K2: 观测器增益
    """
    # 计算中间变量
    e1 = 1 - rs * T / ls
    e2 = 1
    e1_obs = e1 / f
    e2_obs = e2 / f
    
    # 计算增益
    K1 = (e1_obs + e2_obs - 2) / T + rs / ls
    K2 = ls * (1 - e1_obs - e2_obs + e1_obs * e2_obs) / (T * T)
    
    return K1, K2


# 示例使用
if __name__ == "__main__":
    # 输入参数
    rs = 0.11       # 定子电阻 (Ω)
    ls = 0.00097  # 定子电感 (H)
    T = 1e-4        # PWM周期 (s)
    f = 6         # 极点放置系数 (5~10)
    
    K1, K2 = calculate_st_gains(rs, ls, T, f)
    
    print(f"观测器增益:")
    print(f"K1 = {K1:.2f}")
    print(f"K2 = {K2:.2e}")