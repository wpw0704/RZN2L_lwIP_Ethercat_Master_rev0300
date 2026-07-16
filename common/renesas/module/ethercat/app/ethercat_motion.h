#ifndef RZN2L_LWIP_ETHERCAT_MASTER_REV0300_ETHERCAT_MOTION_H
#define RZN2L_LWIP_ETHERCAT_MASTER_REV0300_ETHERCAT_MOTION_H

#include "osal.h"
#include "ethercat_master.h"

typedef enum {
    CSP_MOTION_MODE_IDLE = 0,        // 空闲，不更新目标位置
    CSP_MOTION_MODE_JOG,             // 点动模式：按键/条件触发，目标位置连续加减
    CSP_MOTION_MODE_MOVE_POSITION, /* 通用位置运动，绝对和相对共用 */
    CSP_MOTION_MODE_HOME_ZERO,       // 回零模式：回到软件零点/机械零点
    CSP_MOTION_MODE_RECIP,           // 往返模式：在两个位置之间往复
    CSP_MOTION_MODE_SEQUENCE,        // 序列模式：按多个目标点依次运动
    CSP_MOTION_MODE_STOP,            // 停止模式：保持当前位置或按减速度停下
    CSP_MOTION_MODE_FAULT,           // 运动异常
} csp_motion_mode_t;

typedef struct {
    csp_motion_mode_t mode;          // 当前运动模式
    csp_motion_mode_t last_mode;     // 上一次运动模式，用于检测模式切换

    uint8_t enable;                  // 运动使能，1 允许运动，0 停止更新目标
    uint8_t busy;                    // 正在运动
    uint8_t done;                    // 当前运动完成
    uint8_t error;                   // 运动错误标志

    int32 command_pos;               // 当前命令位置，最终写入 TargetPos
    int32 zero_pos;                  // 软件零点位置
} csp_motion_ctrl_t;

#define CSP_PDO_PERIOD_S    (0.002f)   // PDO 周期，当前为 2ms

/*
 * 机械参数结构体。
 * 后续上位机可以下发这些参数，用来适配不同导程、齿轮比、电机编码器分辨率。
 */
typedef struct {
    float motor_counts_per_rev;        // 电机轴转一圈对应的 counts，例如 262144
    float lead_mm_per_load_rev;        // 负载侧转一圈对应的直线位移，单位 mm，例如 50
    float gear_ratio_motor_per_load;   // 齿轮比：电机圈数 / 负载圈数，直连为 1
} csp_mech_param_t;

/*
 * 通用 CSP 位置轨迹结构体。
 * 所有运动模式最终都可以转换成：
 * 当前命令位置 command_pos -> 目标位置 target_pos
 */
typedef struct {
    uint8_t running; // 1 表示轨迹正在运行
    uint8_t done; // 1 表示轨迹到达目标

    int32 command_pos; // 当前主站命令位置，单位 counts，每周期写给 TargetPos
    int32 target_pos; // 目标位置，单位 counts

    float velocity; // 当前轨迹速度，单位 counts/s，内部始终使用正值
    float max_velocity; // 最大速度，单位 counts/s
    float accel; // 加速度，单位 counts/s^2
} csp_traj_t;

/* 位置指令使用的坐标类型 */
typedef enum {
    CSP_POSITION_ABSOLUTE = 0, /* 机械零点为0的绝对位置 */
    CSP_POSITION_RELATIVE,     /* 相对于命令开始时的位置 */
} csp_position_type_t;

/* 绝对运动和相对运动共用的命令参数 */
typedef struct {
    csp_position_type_t type;
    float position_mm;         /* 绝对位置或相对位移 */
    float velocity_mps;
    float accel_mps2;
    uint8_t pending;           /* 新命令等待轨迹模块接收 */
    uint8_t active;            /* 当前轨迹正在运行 */
    int32 target_counts;       /* 最终绝对编码器目标位置 */
} csp_position_command_t;

/* 根据真实机构修改最大行程 */
#define CSP_MACHINE_MAX_POSITION_MM  (100000.0f)
#define CSP_MAX_VELOCITY_MPS          (100.0f)
#define CSP_MAX_ACCEL_MPS2            (50.0f)

// 设置运动模式
void ethercat_csp_motion_mode_set(csp_motion_mode_t mode);

void ethercat_csp_motion_process(void);

int ethercat_csp_mech_param_set(float motor_counts_per_rev,
                                 float lead_mm_per_load_rev,
                                 float gear_ratio_motor_per_load);

/* 运动到机械零点为基准的绝对位置，位置单位mm */
int ethercat_csp_move_abs_start_mm(float target_mm,
                                   float velocity_mps,
                                   float accel_mps2);

/* 从命令开始时的位置移动指定距离，delta_mm允许为负数 */
int ethercat_csp_move_rel_start_mm(float delta_mm,
                                   float velocity_mps,
                                   float accel_mps2);
#endif //RZN2L_LWIP_ETHERCAT_MASTER_REV0300_ETHERCAT_MOTION_H
