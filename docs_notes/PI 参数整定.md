1.串联PI结构

![image-20260310152412312](C:\Users\30495\AppData\Roaming\Typora\typora-user-images\image-20260310152412312.png)

传递函数：
$$
G(s)=K_p+\frac{K_pK_i}{s}=\frac{K_p(s+K_i)}{s}
=K_p\frac{1}{s}(s+K_i)
=K_p\cdot\frac{K_i}{s}\cdot\left(1+\frac{s}{K_i}\right)
$$
