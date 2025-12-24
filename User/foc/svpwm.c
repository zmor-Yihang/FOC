#include "svpwm.h"

// /**
//  * @brief  SVPWM调制函数 (中心对齐PWM，带零序分量注入)
//  * @param  u_abc - 输入的三相调制电压 (绝对电压值, 单位: V)
//  * @retval  duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
//  */
// abc_t svpwm_update(abc_t u_abc)
// {
//     abc_t duty;
//     float u_max, u_min, u_zero;

//     u_abc.a /= (U_DC * 0.5f);
//     u_abc.b /= (U_DC * 0.5f);
//     u_abc.c /= (U_DC * 0.5f);

//     u_max = u_abc.a;
//     if (u_abc.b > u_max)
//         u_max = u_abc.b;
//     if (u_abc.c > u_max)
//         u_max = u_abc.c;

//     u_min = u_abc.a;
//     if (u_abc.b < u_min)
//         u_min = u_abc.b;
//     if (u_abc.c < u_min)
//         u_min = u_abc.c;

//     /* 计算零序分量 (min-max方法) */
//     u_zero = -0.5f * (u_max + u_min);

//     /* 注入零序分量后，将电压从 [-1, 1] 映射到占空比 [0, 1]*/
//     duty.a = (u_abc.a + u_zero) * 0.5f + 0.5f;
//     duty.b = (u_abc.b + u_zero) * 0.5f + 0.5f;
//     duty.c = (u_abc.c + u_zero) * 0.5f + 0.5f;

//     // 占空比限幅保护, 防止过调制
//     if (duty.a > 1.0f) duty.a = 1.0f;
//     else if (duty.a < 0.0f) duty.a = 0.0f;

//     if (duty.b > 1.0f) duty.b = 1.0f;
//     else if (duty.b < 0.0f) duty.b = 0.0f;

//     if (duty.c > 1.0f) duty.c = 1.0f;
//     else if (duty.c < 0.0f) duty.c = 0.0f;

//     return duty;
// }

/**
 * @brief  标准SVPWM调制函数 (七段式，中心对齐PWM)
 * @param  u_alpha - α轴电压 (V)
 * @param  u_beta  - β轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_update(alphabeta_t u_alphabeta)
{
    abc_t duty;
    uint8_t bSector;
    float wX, wY, wZ, wUAlpha, wUBeta;
    float hTimePhA = 0, hTimePhB = 0, hTimePhC = 0;
    
    /* ============ 1. 计算中间变量 ============ */
    // 归一化到周期T（这里用1.0表示满周期）
    wUAlpha = u_alphabeta.alpha * 1.732051f / U_DC;  // Valpha * sqrt(3) / Udc
    wUBeta  = -u_alphabeta.beta / U_DC;               // -Vbeta / Udc (注意负号!)
    
    wX = wUBeta;
    wY = (wUBeta + wUAlpha) / 2.0f;
    wZ = (wUBeta - wUAlpha) / 2.0f;
    
    /* ============ 2. 扇区判断 ============ */
    if (wY < 0)
    {
        if (wZ < 0)
        {
            bSector = 5;
        }
        else  // wZ >= 0
        {
            if (wX <= 0)
            {
                bSector = 4;
            }
            else  // wX > 0
            {
                bSector = 3;
            }
        }
    }
    else  // wY >= 0
    {
        if (wZ >= 0)
        {
            bSector = 2;
        }
        else  // wZ < 0
        {
            if (wX <= 0)
            {
                bSector = 6;
            }
            else  // wX > 0
            {
                bSector = 1;
            }
        }
    }
    
    /* ============ 3. 根据扇区计算三相占空比 ============ */
    switch (bSector)
    {
        case 1:
        case 4:
            hTimePhA = (1.0f + wX - wZ) / 2.0f;
            hTimePhB = hTimePhA + wZ;
            hTimePhC = hTimePhB - wX;
            break;
            
        case 2:
        case 5:
            hTimePhA = (1.0f + wY - wZ) / 2.0f;
            hTimePhB = hTimePhA + wZ;
            hTimePhC = hTimePhA - wY;
            break;
            
        case 3:
        case 6:
            hTimePhA = (1.0f - wX + wY) / 2.0f;
            hTimePhC = hTimePhA - wY;
            hTimePhB = hTimePhC + wX;
            break;
            
        default:
            hTimePhA = 0.5f;
            hTimePhB = 0.5f;
            hTimePhC = 0.5f;
            break;
    }
    
    /* ============ 4. 限幅保护 ============ */
    duty.a = (hTimePhA > 1.0f) ? 1.0f : ((hTimePhA < 0.0f) ? 0.0f : hTimePhA);
    duty.b = (hTimePhB > 1.0f) ? 1.0f : ((hTimePhB < 0.0f) ? 0.0f : hTimePhB);
    duty.c = (hTimePhC > 1.0f) ? 1.0f : ((hTimePhC < 0.0f) ? 0.0f : hTimePhC);
    
    return duty;
}
