#include <reg52.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* 硬件接口定义 */
sbit PUL  = P1^0;    // 电机1脉冲信号(X轴)
sbit DIR  = P1^1;    // 电机1方向信号(X轴)
sbit ENA  = P1^2;    // 电机1使能信号（低有效）
sbit PUL2 = P1^7;    // 电机2脉冲信号(Y轴)
sbit DIR2 = P1^6;    // 电机2方向信号(Y轴)

/* 电机参数配置 */
#define DEFAULT_STEPS_M1 3200  // 电机1默认步数/圈
#define DEFAULT_STEPS_M2 3200  // 电机2默认步数/圈

/* 全局变量 - 优化内存分配 */
typedef struct {
    unsigned int step_count;
    unsigned int target_steps;
    unsigned char direction;
} MotorControl;

typedef struct {
    int x0, y0;      // 起点坐标
    int x1, y1;      // 终点坐标
    int dx, dy;      // 坐标差值
    int sx, sy;      // 步进方向
    int err;         // 误差项
    int x, y;        // 当前坐标
    unsigned int step_count;  // 当前步数
    unsigned int total_steps; // 总步数
    unsigned char active;     // 插补激活标志
} LineInterpolator;

/* 圆弧插补结构体 */
typedef struct {
    int x0, y0;      // 起点坐标
    int x1, y1;      // 终点坐标
    int x, y;        // 当前坐标
    int r;           // 半径
    int f;           // 偏差值
    unsigned int step_count;  // 当前步数
    unsigned int total_steps; // 总步数
    unsigned char active;     // 插补激活标志
    unsigned char quadrant;   // 当前象限
    unsigned char direction;  // 方向(0:逆时针,1:顺时针)
} ArcInterpolator;

/* 关键性能变量放在idata */
idata MotorControl motor1 = {0, DEFAULT_STEPS_M1, 1};
idata MotorControl motor2 = {0, DEFAULT_STEPS_M2, 1};
bit running = 0;     // 运行状态标志

/* 大块数据放在xdata */
xdata LineInterpolator line_interp = {0};
xdata ArcInterpolator arc_interp = {0};

/* 串口通信配置 */
#define UART_BUF_SIZE 32  // 适当增大缓冲区，放在xdata
xdata unsigned char uart_buf[UART_BUF_SIZE];
unsigned char buf_index = 0;

/******************** 函数声明 ********************/
void Timer_Init(void);
void UART_Init(void);
void UART_SendByte(unsigned char dat);
void UART_SendString(const char *s);
void Start_Motion(void);
void Stop_Motion(void);
void Process_Command(const char *cmd);
void LineInterp_Init(int x0, int y0, int x1, int y1);
bit LineInterp_Step(void);
void Start_LineInterpolation(int x0, int y0, int x1, int y1);
void Stop_LineInterpolation(void);
void ArcInterp_Init(int x0, int y0, int x1, int y1, int r, unsigned char direction);
bit ArcInterp_Step(void);
void Start_ArcInterpolation(int x0, int y0, int x1, int y1, int r, unsigned char direction);
void Stop_ArcInterpolation(void);
void Determine_Quadrant(void);
int Calculate_TotalSteps(int x0, int y0, int x1, int y1);

/******************** 初始化函数 ********************/
void Timer_Init(void)
{
    TMOD = 0x21;    // T0-模式1(16位), T1-模式2(8位自动重装)
    TH0 = 0xFF;     // 定时0.5ms@12MHz
    TL0 = 0x9C;
    ET0 = 1;        // 允许T0中断
    TR0 = 0;        // 初始停止
    
    TH1 = 0xFD;     // 波特率9600@12MHz
    TL1 = 0xFD;
}

void UART_Init(void)
{ 
    SCON = 0x50;    // 串口模式1,允许接收
    TR1 = 1;        // 启动波特率发生器
    ES = 1;         // 允许串口中断
    EA = 1;         // 开总中断
}

/******************** 串口通信函数 ********************/
void UART_SendByte(unsigned char dat)
{
    SBUF = dat;
    while(!TI);
    TI = 0;
}

void UART_SendString(const char *s)
{
    while(*s) {
        UART_SendByte(*s++);
    }
}

/******************** 直线插补函数 ********************/
void LineInterp_Init(int x0, int y0, int x1, int y1)
{
    line_interp.x0 = x0;
    line_interp.y0 = y0;
    line_interp.x1 = x1;
    line_interp.y1 = y1;
    line_interp.x = x0;
    line_interp.y = y0;
    
    // 计算坐标差值
    line_interp.dx = abs(x1 - x0);
    line_interp.dy = abs(y1 - y0);
    
    // 计算总步数(取X或Y方向的最大值)
    line_interp.total_steps = (line_interp.dx > line_interp.dy) ? line_interp.dx : line_interp.dy;
    line_interp.step_count = line_interp.total_steps;
    
    // 计算步进方向
    line_interp.sx = (x1 > x0) ? 1 : -1;
    line_interp.sy = (y1 > y0) ? 1 : -1;
    
    // 初始化偏差
    line_interp.err = (line_interp.dx > line_interp.dy) ? (line_interp.dx / 2) : (-line_interp.dy / 2);
    line_interp.active = 1;
}

bit LineInterp_Step(void)
{
    // 终点判断
    if (line_interp.step_count == 0) {
        line_interp.active = 0;
        return 0;
    }
    
    // 偏差判别
    if (line_interp.err >= 0) {
        // X方向进给
        line_interp.x += line_interp.sx;
        line_interp.err -= line_interp.dy;
    } else {
        // Y方向进给
        line_interp.y += line_interp.sy;
        line_interp.err += line_interp.dx;
    }
    
    line_interp.step_count--;
    return 1;
}

/******************** 圆弧插补函数 ********************/
void Determine_Quadrant(void)
{
    /* 确定圆弧所在象限
     * 根据起点和终点坐标判断圆弧所在的象限
     * 参考逐点比较法象限处理原理:cite[7]
     */
    if(arc_interp.x >= 0 && arc_interp.y >= 0) {
        arc_interp.quadrant = 1;  // 第一象限
    } else if(arc_interp.x < 0 && arc_interp.y >= 0) {
        arc_interp.quadrant = 2;  // 第二象限
    } else if(arc_interp.x < 0 && arc_interp.y < 0) {
        arc_interp.quadrant = 3;  // 第三象限
    } else {
        arc_interp.quadrant = 4;  // 第四象限
    }
}

int Calculate_TotalSteps(int x0, int y0, int x1, int y1)
{
    /* 估算圆弧总步数
     * 使用简单的弦长近似计算
     */
    float angle_start = atan2(y0, x0);
    float angle_end = atan2(y1, x1);
    float angle_diff = fabs(angle_end - angle_start);
    if(angle_diff > 3.1415926) {
        angle_diff = 2 * 3.1415926 - angle_diff;
    }
    return (int)(angle_diff * arc_interp.r * 2);  // 2倍超量估算
}

void ArcInterp_Init(int x0, int y0, int x1, int y1, int r, unsigned char direction)
{
    /* 圆弧插补初始化
     * 参考逐点比较法圆弧插补原理:cite[3]:cite[4]
     */
    arc_interp.x0 = x0;
    arc_interp.y0 = y0;
    arc_interp.x1 = x1;
    arc_interp.y1 = y1;
    arc_interp.x = x0;
    arc_interp.y = y0;
    arc_interp.r = r;
    arc_interp.direction = direction;
    
    // 计算总步数(估算)
    arc_interp.total_steps = Calculate_TotalSteps(x0, y0, x1, y1);
    arc_interp.step_count = arc_interp.total_steps;
    
    // 初始偏差值
    arc_interp.f = (arc_interp.x * arc_interp.x) + (arc_interp.y * arc_interp.y) - (arc_interp.r * arc_interp.r);
    
    // 确定初始象限
    Determine_Quadrant();
    
    arc_interp.active = 1;
}

bit ArcInterp_Step(void)
{
    /* 圆弧插补单步执行
     * 参考逐点比较法圆弧插补算法:cite[1]:cite[4]
     */
    int x_temp, y_temp;
    int f_temp1, f_temp2;
    
    // 终点判断
    if(arc_interp.step_count == 0 || 
       (arc_interp.x == arc_interp.x1 && arc_interp.y == arc_interp.y1)) {
        arc_interp.active = 0;
        return 0;
    }
    
    // 根据象限和方向确定进给方向
    switch(arc_interp.quadrant) {
        case 1:  // 第一象限
            if(arc_interp.direction == 0) {  // 逆时针
                // 第一象限逆圆插补规则:cite[5]
                if(arc_interp.f >= 0) {
                    // -X方向进给
                    arc_interp.x--;
                    arc_interp.f -= 2 * arc_interp.x + 1;
                } else {
                    // +Y方向进给
                    arc_interp.y++;
                    arc_interp.f += 2 * arc_interp.y + 1;
                }
            } else {  // 顺时针
                // 第一象限顺圆插补规则:cite[3]
                if(arc_interp.f >= 0) {
                    // -Y方向进给
                    arc_interp.y--;
                    arc_interp.f -= 2 * arc_interp.y + 1;
                } else {
                    // +X方向进给
                    arc_interp.x++;
                    arc_interp.f += 2 * arc_interp.x + 1;
                }
            }
            break;
            
        case 2:  // 第二象限
            if(arc_interp.direction == 0) {  // 逆时针
                if(arc_interp.f >= 0) {
                    // -Y方向进给
                    arc_interp.y--;
                    arc_interp.f -= 2 * arc_interp.y + 1;
                } else {
                    // -X方向进给
                    arc_interp.x--;
                    arc_interp.f += 2 * arc_interp.x + 1;
                }
            } else {  // 顺时针
                if(arc_interp.f >= 0) {
                    // +X方向进给
                    arc_interp.x++;
                    arc_interp.f += 2 * arc_interp.x + 1;
                } else {
                    // -Y方向进给
                    arc_interp.y--;
                    arc_interp.f -= 2 * arc_interp.y + 1;
                }
            }
            break;
            
        case 3:  // 第三象限
            if(arc_interp.direction == 0) {  // 逆时针
                if(arc_interp.f >= 0) {
                    // +X方向进给
                    arc_interp.x++;
                    arc_interp.f += 2 * arc_interp.x + 1;
                } else {
                    // -Y方向进给
                    arc_interp.y--;
                    arc_interp.f -= 2 * arc_interp.y + 1;
                }
            } else {  // 顺时针
                if(arc_interp.f >= 0) {
                    // +Y方向进给
                    arc_interp.y++;
                    arc_interp.f += 2 * arc_interp.y + 1;
                } else {
                    // +X方向进给
                    arc_interp.x++;
                    arc_interp.f += 2 * arc_interp.x + 1;
                }
            }
            break;
            
        case 4:  // 第四象限
            if(arc_interp.direction == 0) {  // 逆时针
                if(arc_interp.f >= 0) {
                    // +Y方向进给
                    arc_interp.y++;
                    arc_interp.f += 2 * arc_interp.y + 1;
                } else {
                    // +X方向进给
                    arc_interp.x++;
                    arc_interp.f += 2 * arc_interp.x + 1;
                }
            } else {  // 顺时针
                if(arc_interp.f >= 0) {
                    // -X方向进给
                    arc_interp.x--;
                    arc_interp.f -= 2 * arc_interp.x + 1;
                } else {
                    // +Y方向进给
                    arc_interp.y++;
                    arc_interp.f += 2 * arc_interp.y + 1;
                }
            }
            break;
    }
    
    // 更新象限
    Determine_Quadrant();
    
    arc_interp.step_count--;
    return 1;
}

/******************** 电机控制函数 ********************/
void Stop_Motion(void)
{
    TR0 = 0;
    ENA = 1;
    running = 0;
    UART_SendString("STOP\r\n");
}

void Start_LineInterpolation(int x0, int y0, int x1, int y1)
{
    LineInterp_Init(x0, y0, x1, y1);
    // 使能电机
    ENA = 0;
    running = 1;
    TR0 = 1;
    UART_SendString("LINE INTERP START\r\n");
}

void Stop_LineInterpolation(void)
{
    TR0 = 0;
    ENA = 1;
    running = 0;
    line_interp.active = 0;
    UART_SendString("LINE INTERP STOP\r\n");
}

void Start_ArcInterpolation(int x0, int y0, int x1, int y1, int r, unsigned char direction)
{
    ArcInterp_Init(x0, y0, x1, y1, r, direction);
    // 使能电机
    ENA = 0;
    running = 1;
    TR0 = 1;
    UART_SendString("ARC INTERP START\r\n");
}

void Stop_ArcInterpolation(void)
{
    TR0 = 0;
    ENA = 1;
    running = 0;
    arc_interp.active = 0;
    UART_SendString("ARC INTERP STOP\r\n");
}

/******************** 命令处理函数 ********************/
void Process_Command(const char *cmd)
{
    int x0, y0, x1, y1, r;
    unsigned char direction;

    if(strcmp(cmd, "STOP") == 0) {
        if(line_interp.active) {
            Stop_LineInterpolation();
        } else if(arc_interp.active) {
            Stop_ArcInterpolation();
        } else {
            Stop_Motion();
        }
    }
    else if(sscanf(cmd, "LINE %d %d %d %d", &x0, &y0, &x1, &y1) == 4) {
        Start_LineInterpolation(x0, y0, x1, y1);
    }
    else if(sscanf(cmd, "ARC_CW %d %d %d %d %d", &x0, &y0, &x1, &y1, &r) == 5) {
        Start_ArcInterpolation(x0, y0, x1, y1, r, 1);  // 顺时针
    }
    else if(sscanf(cmd, "ARC_CCW %d %d %d %d %d", &x0, &y0, &x1, &y1, &r) == 5) {
        Start_ArcInterpolation(x0, y0, x1, y1, r, 0);  // 逆时针
    }
    else {
        UART_SendString("ERROR: Unknown command\r\n");
    }
}
/******************** 中断服务函数 ********************/
void Timer0_ISR() interrupt 1
{
    static int last_x = 0, last_y = 0;
    int dx, dy;

    TH0 = 0xFF;     // 重装定时值
    TL0 = 0x9C;

    // 优先处理圆弧插补
    if (arc_interp.active) {
			
        // 记录上次坐标
        last_x = arc_interp.x;
        last_y = arc_interp.y;
        
        // 执行圆弧插补步进
        if (!ArcInterp_Step()) {
            arc_interp.active = 0;
            running = 0;
            TR0 = 0;
            UART_SendString("ARC INTERP DONE\r\n");
						return;
				}
				// 计算坐标变化量
					dx = arc_interp.x - last_x;
					dy = arc_interp.y - last_y;
					
					// 电机1步进(X轴)
					if (dx != 0) {
							DIR = (dx > 0) ? 1 : 0;
							PUL = ~PUL;
					}
					
					// 电机2步进(Y轴)
					if (dy != 0) {
							DIR2 = (dy > 0) ? 1 : 0;
							PUL2 = ~PUL2;
					}
					
					return;
			}
		// 处理直线插补
		if (line_interp.active) {
				// 记录上次坐标
				last_x = line_interp.x;
				last_y = line_interp.y;
				
				// 执行插补步进
				if (!LineInterp_Step()) {
						line_interp.active = 0;
						running = 0;
						TR0 = 0;
						UART_SendString("LINE INTERP DONE\r\n");
						return;
				}
				
				// 计算坐标变化量
				dx = line_interp.x - last_x;
				dy = line_interp.y - last_y;
				
				// 电机1步进(X轴)
				if (dx != 0) {
						DIR = (dx > 0) ? 1 : 0;
						PUL = ~PUL;
				}
				
				// 电机2步进(Y轴)
				if (dy != 0) {
						DIR2 = (dy > 0) ? 1 : 0;
						PUL2 = ~PUL2;
				}
				
				return;
		}

		// 原有单轴运动控制保持不变
		if (motor1.step_count > 0) {
				PUL = ~PUL;
				if (!PUL) {
						motor1.step_count--;
						if (motor1.step_count == 0) {
								ENA = 1;
								PUL = 0;
						}
				}
		}

		if (motor2.step_count > 0) {
				PUL2 = ~PUL2;
				if (!PUL2) {
						motor2.step_count--;
						if (motor2.step_count == 0) {
								PUL2 = 0;
						}
				}
		}

		if(motor1.step_count == 0 && motor2.step_count == 0 && running) {
				TR0 = 0;
				running = 0;
				UART_SendString("DONE\r\n");
		}
}



void UART_ISR() interrupt 4
{  
    unsigned char c;
    if(RI) {
        RI = 0;
        c = SBUF;
        
        if(c == '\r' || c == '\n') {
            if(buf_index > 0) {
                uart_buf[buf_index] = '\0';
                Process_Command(uart_buf);
                buf_index = 0;
            }
        }
        else if(buf_index < UART_BUF_SIZE-1) {
            uart_buf[buf_index++] = c;
        }
        else {
            buf_index = 0;  // 缓冲区溢出保护
            UART_SendString("BUF FULL\r\n");
        }
    }
    
    if(TI) {
        TI = 0;  // 清除发送中断标志
    }
}

/******************** 主函数 ********************/
void main()
{  
    // 硬件初始化
    Timer_Init();
    UART_Init();
    Stop_Motion();  // 确保电机初始为停止状态
    
    // 系统启动信息
    UART_SendString("READY\r\n");
		UART_SendString("Commands:\r\n");
		UART_SendString("LINE x0 y0 x1 y1 - Line interpolation\r\n");
		UART_SendString("ARC_CW x0 y0 x1 y1 r - Clockwise arc\r\n");
		UART_SendString("ARC_CCW x0 y0 x1 y1 r - Counter-clockwise arc\r\n");
		UART_SendString("STOP - Stop motion\r\n");
    
    // 主循环
    while(1) {
        // 空闲时进入省电模式
        PCON |= 0x01;  // 进入空闲模式
    }
}