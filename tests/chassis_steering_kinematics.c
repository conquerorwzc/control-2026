/*
 * 舵轮底盘运动学解算的独立校验(host 侧运行, 不参与固件编译)。
 *
 * 目的: chassis.c 里的轮组自转分量符号非常容易写错 —— 而且这类错误在【纯平移时
 *       完全看不出来】, 只有自转才暴露。本文件把同一套公式抄成独立实现, 断言三条
 *       不变量; 改动 chassis.c 的 kSteerCwSign / kSteerPos / atan2 参数顺序后必须重跑。
 *
 * 三条不变量:
 *   1. 刚体自洽: 扣掉整车平移分量后, 各轮速度必须与位置矢量垂直(切向);
 *   2. 往返一致: 逆解算 -> 正解算 必须还原原指令;
 *   3. 方向正确: +x=右移 / +y=前进 / +wz=顺时针 三个轴的舵角期望值。
 *
 * 用法:
 *     cc -O0 -o /tmp/kin_check tests/chassis_steering_kinematics.c -lm && /tmp/kin_check
 *
 * @attention 本文件与 chassis.c 是【两份独立实现】, 必须手工保持同步:
 *            改了 chassis.c 的 kSteerPos / kSteerCwSign / atan2 参数顺序, 这里也要改。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum { LF = 0, LB, RB, RF, WHEEL_COUNT } Idx;

typedef struct { float x, y; } Vec;

/* ---- 与 chassis.c 完全一致的约定 ---- */
#define kSteerCwSign (+1.0f)

static const Vec kSteerPos[WHEEL_COUNT] = {
    [LF] = {.x = -1.0f, .y = +1.0f}, /* 左前 */
    [LB] = {.x = -1.0f, .y = -1.0f}, /* 左后 */
    [RB] = {.x = +1.0f, .y = -1.0f}, /* 右后 */
    [RF] = {.x = +1.0f, .y = +1.0f}, /* 右前 */
};

static const char *NAME[4] = {"LF", "LB", "RB", "RF"};

static float rad2deg(float r) { return r * 57.2957795f; }

/* 逆运动学: 与 SteeringCalculate() 相同 */
static void ik(float vx, float vy, float wz, Vec *v, float *st_deg) {
    for (int i = 0; i < 4; ++i) {
        v[i].x = vx + (+kSteerCwSign * wz * kSteerPos[i].y);
        v[i].y = vy + (-kSteerCwSign * wz * kSteerPos[i].x);
        st_deg[i] = rad2deg(atan2f(v[i].x, v[i].y)); /* 从 +y(前) 量起 */
    }
}

/* ---- 编码器方向约定 (实测: 舵轮在地上逆时针旋转时 ecd 变大) ----
 *
 * 解算约定: st 随【顺时针】增大。
 * 编码器:   随【逆时针】增大。
 * => 两者相反, FK 里必须把编码器角取负才能换算成解算角。
 *
 * 对应到配置: 角度环闭环用的是 total_angle, 要让它随顺时针增大, 必须
 * feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE (把编码器角的符号翻过来)。
 */
#define kEncoderCcwPositive 0 /* 与 chassis.c 的 kRudderEncCcwPositive 保持一致 */

/* 编码器角(度, 逆时针为正, 0 = 指向正前方) -> 解算角 st(度, 顺时针为正) */
static float enc_to_st_deg(float enc_deg) {
    float a = enc_deg;
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return kEncoderCcwPositive ? -a : a;
}

/* 解算角 st(度) -> 该姿态下应读到的编码器角(度, 逆时针为正) */
static float st_deg_to_enc(float st_deg) { return kEncoderCcwPositive ? -st_deg : st_deg; }

/* 正运动学: 与 ForwardKinematicCal() 相同 */
static void fk(const Vec *v, float *vx, float *vy, float *wz) {
    float ox[4], oy[4];
    for (int i = 0; i < 4; ++i) {
        float speed = sqrtf(v[i].x * v[i].x + v[i].y * v[i].y);
        float st = atan2f(v[i].x, v[i].y);
        ox[i] = speed * sinf(st);
        oy[i] = speed * cosf(st);
    }
    *vx = (ox[0] + ox[3] + ox[1] + ox[2]) / 4.0f;
    *vy = (oy[0] + oy[3] + oy[1] + oy[2]) / 4.0f;
    float s = 0.0f;
    for (int i = 0; i < 4; ++i) s += ox[i] * kSteerPos[i].y - oy[i] * kSteerPos[i].x;
    *wz = s / 8.0f;
}

static int failures = 0;
static void check(const char *what, float got, float want, float tol) {
    int ok = fabsf(got - want) <= tol;
    if (!ok) failures++;
    printf("    %-42s got %9.3f  want %9.3f   %s\n", what, got, want, ok ? "OK" : "<<< FAIL");
}

int main(void) {
    printf("=== 1. 刚体自洽性: 扣掉平移后 r_i · (v_i - vbar) 必须为 0 (切向) ===\n");
    const float cases[][3] = {
        {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {5000, -3000, 12000}, {-2000, 4000, -8000},
    };
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        Vec v[4]; float st[4];
        ik(cases[c][0], cases[c][1], cases[c][2], v, st);
        float mx = (v[0].x + v[1].x + v[2].x + v[3].x) / 4.0f;
        float my = (v[0].y + v[1].y + v[2].y + v[3].y) / 4.0f;
        float worst = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float dot = (v[i].x - mx) * kSteerPos[i].x + (v[i].y - my) * kSteerPos[i].y;
            if (fabsf(dot) > fabsf(worst)) worst = dot;
        }
        /* 纯平移时自转分量为 0, 自然满足; 这一条同时覆盖平移与自转两种情形 */
        printf("    case vx=%7.0f vy=%7.0f wz=%7.0f -> max|r.(v-vbar)| = %.6f  %s\n", cases[c][0],
               cases[c][1], cases[c][2], fabsf(worst), fabsf(worst) < 1e-3f ? "OK" : "<<< FAIL");
        if (fabsf(worst) >= 1e-3f) failures++;
    }

    printf("\n=== 2. 逆解算 -> 正解算 往返一致性 (必须还原原指令) ===\n");
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        Vec v[4]; float st[4], vx, vy, wz;
        ik(cases[c][0], cases[c][1], cases[c][2], v, st);
        fk(v, &vx, &vy, &wz);
        printf("    case vx=%7.0f vy=%7.0f wz=%7.0f\n", cases[c][0], cases[c][1], cases[c][2]);
        check("FK_Vx", vx, cases[c][0], 1e-2f);
        check("FK_Vy", vy, cases[c][1], 1e-2f);
        check("FK_Wz", wz, cases[c][2], 1e-2f);
    }

    printf("\n=== 3. 实测期望值 (st 单位: 度, 0=指向正前方) ===\n");
    {
        Vec v[4]; float st[4];

        ik(0, 10000, 0, v, st); /* 左摇杆推前 */
        printf("  +vy 前进:  ");
        for (int i = 0; i < 4; ++i) printf("%s=%7.2f ", NAME[i], st[i]);
        printf("\n");
        for (int i = 0; i < 4; ++i) check(NAME[i], st[i], 0.0f, 1e-3f);

        ik(10000, 0, 0, v, st); /* 左摇杆推右 */
        printf("  +vx 右移:  ");
        for (int i = 0; i < 4; ++i) printf("%s=%7.2f ", NAME[i], st[i]);
        printf("\n");
        for (int i = 0; i < 4; ++i) check(NAME[i], st[i], 90.0f, 1e-3f);

        ik(-10000, 0, 0, v, st);
        printf("  -vx 左移:  ");
        for (int i = 0; i < 4; ++i) printf("%s=%7.2f ", NAME[i], st[i]);
        printf("\n");
        for (int i = 0; i < 4; ++i) check(NAME[i], st[i], -90.0f, 1e-3f);

        ik(0, -10000, 0, v, st);
        printf("  -vy 后退:  ");
        for (int i = 0; i < 4; ++i) printf("%s=%7.2f ", NAME[i], st[i]);
        printf("\n");
        for (int i = 0; i < 4; ++i) check(NAME[i], fabsf(st[i]), 180.0f, 1e-3f);

        ik(0, 0, 10000, v, st);
        printf("  +wz 顺时针(目标角应指向切向): ");
        for (int i = 0; i < 4; ++i) printf("%s=%7.2f ", NAME[i], st[i]);
        printf("\n");
        /* 注意: 舵角目标是【短弧等效角】。LF 的切向是 +45 度, 但从未翻转态去 45 度
           要转 135 度(>100 度阈值), 故 AngleToOptimalAngle 会翻 180 度并让轮子反转,
           等价表示为 45-180 = -135 度。四轮都以"等效短弧角"核对。 */
        /* 从静止(st=0)出发时, 四轮都取短弧路径, 因此目标角 = 各自切向角本身:
           LF 切向 +45, LB -45, RB -135, RF +135 (都没有超过 180 度的换向) */
        check("LF 切向 +45", st[LF], 45.0f, 1e-3f);
        check("LB 切向 -45", st[LB], -45.0f, 1e-3f);
        check("RB 切向 -135", st[RB], -135.0f, 1e-3f);
        check("RF 切向 +135", st[RF], 135.0f, 1e-3f);
    }

    printf("\n=== 4. 自转方向物理校验: 自转分量应为 r 顺时针转 90 度(切向) ===\n");
    {
        Vec v[4]; float st[4];
        ik(0, 0, 10000, v, st);
        for (int i = 0; i < 4; ++i) {
            /* 顺时针(俯视)切向 = 位置矢量顺时针转 90°: (dx,dy) -> (dy,-dx) */
            float tx = kSteerPos[i].y, ty = -kSteerPos[i].x;
            float dot = v[i].x * tx + v[i].y * ty;
            float nv = sqrtf(v[i].x * v[i].x + v[i].y * v[i].y);
            float nt = sqrtf(tx * tx + ty * ty);
            float cosang = dot / (nv * nt);
            printf("    %s: v=(%8.1f,%8.1f)  顺时针切向=(%5.1f,%5.1f)  cos=%+.4f  %s\n", NAME[i], v[i].x,
                   v[i].y, tx, ty, cosang, cosang > 0.9999f ? "OK" : "<<< FAIL");
            if (cosang <= 0.9999f) failures++;
        }

        /* 不变量: 扣掉整车平移分量后必须与位置矢量垂直 */
        float mx = (v[0].x + v[1].x + v[2].x + v[3].x) / 4.0f;
        float my = (v[0].y + v[1].y + v[2].y + v[3].y) / 4.0f;
        for (int i = 0; i < 4; ++i) {
            float dot = (v[i].x - mx) * kSteerPos[i].x + (v[i].y - my) * kSteerPos[i].y;
            printf("    %s: r·(v-v̄) = %+.4f  %s\n", NAME[i], dot, fabsf(dot) < 1e-3f ? "OK" : "<<< FAIL");
            if (fabsf(dot) >= 1e-3f) failures++;
        }
    }

    printf("\n=== 5. 组合运动样例 (平移+自转) ===\n");
    {
        Vec v[4]; float st[4], vx, vy, wz;
        ik(8000, 6000, 5000, v, st);
        printf("    vx=8000 vy=6000 wz=5000 -> st: ");
        for (int i = 0; i < 4; ++i) printf("%s=%.1f ", NAME[i], st[i]);
        printf("\n");
        fk(v, &vx, &vy, &wz);
        check("round-trip FK_Vx", vx, 8000.0f, 1e-2f);
        check("round-trip FK_Vy", vy, 6000.0f, 1e-2f);
        check("round-trip FK_Wz", wz, 5000.0f, 1e-2f);
    }

    printf("\n=== 6. transform: offset_angle=0 必须原样透传 ===\n");
    {
        for (int c = 0; c < 4; ++c) {
            float ox = (c == 1) ? -7000.0f : 7000.0f;
            float oy = (c == 2) ? -5000.0f : 5000.0f;
            float t = 0.0f;
            float ct = cosf(t), stt = sinf(t);
            float cvx = ox * ct - oy * stt;
            float cvy = ox * stt + oy * ct;
            printf("    offset=0 (vx,vy)=(%7.0f,%7.0f) -> (%7.0f,%7.0f)  %s\n", ox, oy, cvx, cvy,
                   (fabsf(cvx - ox) < 1e-3f && fabsf(cvy - oy) < 1e-3f) ? "OK" : "<<< FAIL");
            if (!(fabsf(cvx - ox) < 1e-3f && fabsf(cvy - oy) < 1e-3f)) failures++;
        }
    }

    printf("\n=== 7. 编码器方向贯穿: 指令 -> 电机姿态(ecd) -> FK 必须还原指令 ===\n");
    printf("    约定: st 顺时针为正, 编码器逆时针为正 (kEncoderCcwPositive=%d)\n", kEncoderCcwPositive);
    {
        const float cases2[][3] = {{0, 10000, 0}, {10000, 0, 0}, {0, 0, 10000}, {5000, -3000, 12000}};
        for (unsigned c = 0; c < sizeof(cases2) / sizeof(cases2[0]); ++c) {
            Vec v[4]; float st[4];
            ik(cases2[c][0], cases2[c][1], cases2[c][2], v, st);

            /* 电机被闭环到 st_i => 物理姿态对应编码器角 st_deg_to_enc(st_i)
               => FK 读回该编码器角, 换算回解算角, 再解出底盘速度 */
            float ox[4], oy[4];
            for (int i = 0; i < 4; ++i) {
                float enc_deg = st_deg_to_enc(st[i]);
                float st_measured = enc_to_st_deg(enc_deg);
                float speed = sqrtf(v[i].x * v[i].x + v[i].y * v[i].y);
                ox[i] = speed * sinf(st_measured * 0.01745329252f);
                oy[i] = speed * cosf(st_measured * 0.01745329252f);
            }
            float vx = (ox[0] + ox[3] + ox[1] + ox[2]) / 4.0f;
            float vy = (oy[0] + oy[3] + oy[1] + oy[2]) / 4.0f;
            float s = 0.0f;
            for (int i = 0; i < 4; ++i) s += ox[i] * kSteerPos[i].y - oy[i] * kSteerPos[i].x;
            float wz = s / 8.0f;

            printf("    cmd (%7.0f,%7.0f,%7.0f)\n", cases2[c][0], cases2[c][1], cases2[c][2]);
            check("FK_Vx", vx, cases2[c][0], 1e-2f);
            check("FK_Vy", vy, cases2[c][1], 1e-2f);
            check("FK_Wz", wz, cases2[c][2], 1e-2f);
        }
    }

    printf("\n=== 8. 实车判据: 右移时舵轮应指向车头右侧 ===\n");
    {
        Vec v[4]; float st[4];
        ik(10000, 0, 0, v, st);
        for (int i = 0; i < 4; ++i) {
            /* st=+90 表示指向 +x(右)。换算成编码器角后, 逆时针为正的编码器应读到 -90。
               若上板实测轮子指向【左】, 说明 feedback_reverse_flag 取反了。 */
            float enc = st_deg_to_enc(st[i]);
            printf("    %s: st=%+.1f => 该姿态编码器角 %+.1f (指向右侧)\n", NAME[i], st[i], enc);
        }
        printf("    上板核对: 只推左摇杆向右, 四个轮子应全部指向车头【右侧】\n");
    }

    printf("\n=== 9. 世界系 <-> 底盘系变换 (虚拟云台/自旋平移依赖它) ===\n");
    {
        /**
         * 对应 chassis.c 的 TransformCommandToChassisFrame():
         *     chassis_vx = vx*cos(th) - vy*sin(th)
         *     chassis_vy = vx*sin(th) + vy*cos(th)
         * th = offset_angle = "底盘相对世界系(上电朝向)转过的角", 顺时针为正。
         *
         * 核心不变量: 车体转过 th 之后, 把底盘系速度用 R(-th) 转回世界系,
         * 必须【恒等于】最初给的世界系指令 —— 也就是"无论车怎么转,
         * 世界系平移方向不变"。这正是自旋平移要的效果。
         */
        const float th_list[] = {0.0f, 90.0f, -90.0f, 180.0f, 37.0f};
        for (unsigned k = 0; k < sizeof(th_list) / sizeof(th_list[0]); ++k) {
            const float th = th_list[k];
            const float c = cosf(th * 0.01745329252f), s = sinf(th * 0.01745329252f);

            /* 世界系指令: 前进 8000 + 右移 3000 */
            const float wvx = 3000.0f, wvy = 8000.0f;
            const float bvx = wvx * c - wvy * s;   /* 底盘系 */
            const float bvy = wvx * s + wvy * c;

            /* 用 R(-th) 转回世界系 */
            const float rvx = bvx * c + bvy * s;
            const float rvy = -bvx * s + bvy * c;

            printf("    th=%+7.1f  world(%.0f,%.0f) -> body(%.1f,%.1f) -> world(%.1f,%.1f)\n", th, wvx, wvy, bvx,
                   bvy, rvx, rvy);
            check("  回到世界系 vx", rvx, wvx, 1.0f);
            check("  回到世界系 vy", rvy, wvy, 1.0f);
        }

        /* th=0 必须严格退化为原样透传(回归保护: 之前这个函数会把指令翻 180 度) */
        {
            const float c = 1.0f, s = 0.0f;
            const float wvx = 1234.0f, wvy = -5678.0f;
            printf("    th=0 透传检查: world(%.0f,%.0f)\n", wvx, wvy);
            check("  th=0 不翻转 vx", wvx * c - wvy * s, wvx, 1e-3f);
            check("  th=0 不翻转 vy", wvx * s + wvy * c, wvy, 1e-3f);
        }

        /* 车顺时针转过 90 度: 车头现在朝世界的 +x(右)方向,
           所以"往世界 +y(上电前方)走"在底盘系里应当是【向左】, 即 (-v, 0) */
        {
            const float th = 90.0f;
            const float c = cosf(th * 0.01745329252f), s = sinf(th * 0.01745329252f);
            const float wvx = 0.0f, wvy = 10000.0f;  /* 世界系: 纯前进 */
            const float bvx = wvx * c - wvy * s;
            const float bvy = wvx * s + wvy * c;
            printf("    th=+90(车顺时针转90) 纯前进 -> 底盘系(%.1f,%.1f) 应为(-10000,0)\n", bvx, bvy);
            check("  底盘系 vx", bvx, -10000.0f, 1.0f);
            check("  底盘系 vy", bvy, 0.0f, 1.0f);
        }
    }

    printf("\n================ %s (%d failures) ================\n", failures == 0 ? "ALL CHECKS PASSED" : "FAILURES PRESENT",
           failures);
    return failures != 0;
}
