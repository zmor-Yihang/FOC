/**
 * @file test_fast_sin_cos.c
 * @brief fast_sin_cos 精度与性能测试（GCC 独立编译）
 *
 * 编译命令（在 User/test 目录下运行）：
 *   gcc test_fast_sin_cos.c -o test_fast_sin_cos -lm -O2 -march=native -I../../
 *
 * 运行：
 *   ./test_fast_sin_cos        (Linux/MinGW)
 *   test_fast_sin_cos.exe      (Windows)
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <float.h>

/* 引入被测模块 */
#include "../utils/fast_sin_cos.h"

/* ------------------------------------------------------------------ */
/*  配置                                                                */
/* ------------------------------------------------------------------ */
#define BENCH_N       100000000    /* 性能测试循环次数 */
#define ACCURACY_N    1000       /* 精度测试采样数   */
#define PRINT_ROWS    20         /* 精度表格打印行数 */

/* ------------------------------------------------------------------ */
/*  防止编译器优化掉计算结果                                            */
/* ------------------------------------------------------------------ */
static volatile float sink = 0.0f;

/* ------------------------------------------------------------------ */
/*  计时工具（毫秒，使用标准 clock()）                                 */
/* ------------------------------------------------------------------ */
static double get_ms(void)
{
    return (double)clock() * 1000.0 / CLOCKS_PER_SEC;
}

/* ------------------------------------------------------------------ */
/*  精度测试                                                            */
/* ------------------------------------------------------------------ */
static void accuracy_test(void)
{
    printf("=== Accuracy Test  (N=%d, range [-2PI, +4PI]) ===\n\n", ACCURACY_N);
    printf("%-12s  %-11s  %-11s  %-12s  %-11s  %-11s  %-12s\n",
           "Angle(rad)", "sinf", "fast_sin", "sin_err",
           "cosf", "fast_cos", "cos_err");
    printf("%-12s  %-11s  %-11s  %-12s  %-11s  %-11s  %-12s\n",
           "----------", "---------", "---------", "----------",
           "---------", "---------", "----------");

    float max_sin_err = 0.0f, max_cos_err = 0.0f;
    double sum_sin_err = 0.0, sum_cos_err = 0.0;

    /* 覆盖范围：[-2π, +4π]，测试负角度与大角度场景 */
    float range_start = -2.0f * (float)M_PI;
    float range_end   =  4.0f * (float)M_PI;
    float step = (range_end - range_start) / (float)(ACCURACY_N - 1);
    int print_step = ACCURACY_N / PRINT_ROWS;

    for (int i = 0; i < ACCURACY_N; i++)
    {
        float angle = range_start + i * step;

        float ref_s = sinf(angle);
        float ref_c = cosf(angle);
        float fst_s = fast_sin(angle);
        float fst_c = fast_cos(angle);

        float sin_err = fabsf(fst_s - ref_s);
        float cos_err = fabsf(fst_c - ref_c);

        if (sin_err > max_sin_err) max_sin_err = sin_err;
        if (cos_err > max_cos_err) max_cos_err = cos_err;
        sum_sin_err += sin_err;
        sum_cos_err += cos_err;

        if (print_step > 0 && i % print_step == 0)
        {
            printf("%-12.5f  %-11.7f  %-11.7f  %-12.3e  %-11.7f  %-11.7f  %-12.3e\n",
                   angle,
                   ref_s, fst_s, sin_err,
                   ref_c, fst_c, cos_err);
        }
    }

    printf("\n");
    printf("sin  Max absolute error : %.4e\n", max_sin_err);
    printf("sin  Avg absolute error : %.4e\n", sum_sin_err / ACCURACY_N);
    printf("cos  Max absolute error : %.4e\n", max_cos_err);
    printf("cos  Avg absolute error : %.4e\n", sum_cos_err / ACCURACY_N);
    printf("\n");
}

/* ------------------------------------------------------------------ */
/*  性能基准测试（一行辅助宏，减少重复代码）                            */
/* ------------------------------------------------------------------ */
#define BENCH_PRINT(label_ref, label_fast, t_ref_ms, t_fast_ms)        \
    do {                                                                \
        printf("%-16s  %8.1f ms   avg %6.2f ns/call\n",               \
               (label_ref),  (t_ref_ms),                               \
               (t_ref_ms)  * 1e6 / BENCH_N);                           \
        printf("%-16s  %8.1f ms   avg %6.2f ns/call\n",               \
               (label_fast), (t_fast_ms),                              \
               (t_fast_ms) * 1e6 / BENCH_N);                           \
        printf("Speedup           :  %.2fX\n\n",                       \
               (t_ref_ms) / ((t_fast_ms) > 0.0 ? (t_fast_ms) : 1e-9));\
    } while(0)

static void benchmark(void)
{
    printf("=== Performance Benchmark  (N = %d) ===\n\n", BENCH_N);

    double t0, t1, dt_ref, dt_fast;

    /* ---------- sin ---------- */
    t0 = get_ms();
    for (int i = 0; i < BENCH_N; i++)
        sink = sinf((float)i * 1e-6f);
    t1 = get_ms();
    dt_ref = t1 - t0;

    t0 = get_ms();
    for (int i = 0; i < BENCH_N; i++)
        sink = fast_sin((float)i * 1e-6f);
    t1 = get_ms();
    dt_fast = t1 - t0;

    BENCH_PRINT("sinf", "fast_sin", dt_ref, dt_fast);

    /* ---------- cos ---------- */
    t0 = get_ms();
    for (int i = 0; i < BENCH_N; i++)
        sink = cosf((float)i * 1e-6f);
    t1 = get_ms();
    dt_ref = t1 - t0;

    t0 = get_ms();
    for (int i = 0; i < BENCH_N; i++)
        sink = fast_cos((float)i * 1e-6f);
    t1 = get_ms();
    dt_fast = t1 - t0;

    BENCH_PRINT("cosf", "fast_cos", dt_ref, dt_fast);

    /* ---------- sinf+cosf  vs  fast_sin_cos ---------- */
    t0 = get_ms();
    for (int i = 0; i < BENCH_N; i++)
    {
        float a = (float)i * 1e-6f;
        sink = sinf(a) + cosf(a);
    }
    t1 = get_ms();
    dt_ref = t1 - t0;

    t0 = get_ms();
    for (int i = 0; i < BENCH_N; i++)
    {
        float s, c;
        fast_sin_cos((float)i * 1e-6f, &s, &c);
        sink = s + c;
    }
    t1 = get_ms();
    dt_fast = t1 - t0;

    BENCH_PRINT("sinf + cosf", "fast_sin_cos", dt_ref, dt_fast);
}

/* ------------------------------------------------------------------ */
/*  main                                                                */
/* ------------------------------------------------------------------ */
int main(void)
{
    accuracy_test();
    benchmark();
    return 0;
}
