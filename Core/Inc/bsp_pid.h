#ifndef __BSP_PID_H
#define __BSP_PID_H
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
typedef struct
{
        float x_error;  //x的误差值
        float x_intergreal;  //x的积分
        float x_derivative;  //x的微分
        float x_pre_error;   //x的上一次误差
        float x_dt;          //x的测量时间
        float x_output;      //x的输出

        float x_setpoint;    //x的设定值
        //float x_measure;     //x的测量值
        float x_kp;          //kp_x
        float x_ki;          //ki_x
        float x_kd;          //kd_x
}x_pid_InitTypedef;
typedef struct
{
    float y_error;
    float y_intergreal;
    float y_derivative;
    float y_pre_error;
    float y_dt;
    float y_output;

    float y_setpoint;
    //float y_measure;
    float y_kp;
    float y_ki;
    float y_kd;

}y_pid_InitTypedef ;

extern y_pid_InitTypedef y_pid_InitStructure;
extern x_pid_InitTypedef x_pid_InitStructure;


extern char rx_wait[1];
extern char rx_buffer[255];

extern float kp;
extern float ki;
extern float kd;

void pid_init(void);
int x_pid_calculator(x_pid_InitTypedef *xpid,float x_measure);
int y_pid_calculator(y_pid_InitTypedef *ypid,float y_measure);

void RX_Interrupt(UART_HandleTypeDef *huart);
//void TX_Interrupt(UART_HandleTypeDef *huart);

#endif
