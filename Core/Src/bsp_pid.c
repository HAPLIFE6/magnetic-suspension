#include "bsp_pid.h"

    char rx_buffer[255];
    char rx_wait[1];
    uint16_t rx_length=0;
    float kp;
    float ki;
    float kd;
    y_pid_InitTypedef y_pid_InitStructure;
    x_pid_InitTypedef x_pid_InitStructure;
void pid_init(void)
{
    
    x_pid_InitStructure.x_intergreal=0;
    x_pid_InitStructure.x_derivative=0;
    x_pid_InitStructure.x_error=0;
    x_pid_InitStructure.x_dt=1;
    x_pid_InitStructure.x_kp=0;
    x_pid_InitStructure.x_ki=0;
    x_pid_InitStructure.x_kd=0;
    //x_pid_InitStructure.x_measure=;
    x_pid_InitStructure.x_output=0;
    x_pid_InitStructure.x_pre_error=0;
    x_pid_InitStructure.x_setpoint=1918;

    y_pid_InitStructure.y_intergreal=0;
    y_pid_InitStructure.y_derivative=0;
    y_pid_InitStructure.y_error=0;
    y_pid_InitStructure.y_dt=1;
    y_pid_InitStructure.y_kp=0;
    y_pid_InitStructure.y_ki=0;
    y_pid_InitStructure.y_kd=0;
    //y_pid_InitStructure.y_measure=;
    y_pid_InitStructure.y_output=0;
    y_pid_InitStructure.y_pre_error=0;
    y_pid_InitStructure.y_setpoint=2011;
}
int x_pid_calculator(x_pid_InitTypedef *xpid,float x_measure)
{
    xpid->x_error=xpid->x_setpoint-x_measure;
    xpid->x_intergreal=xpid->x_intergreal+(xpid->x_error*xpid->x_dt);
    xpid->x_derivative=(xpid->x_error-xpid->x_pre_error)/xpid->x_dt;
    xpid->x_output=(xpid->x_kp*xpid->x_error)+(xpid->x_ki*xpid->x_intergreal)+(xpid->x_kd*xpid->x_derivative);
    xpid->x_pre_error=xpid->x_error;
    //if(xpid->x_output>300){xpid->x_output=300;}if(xpid->x_output<-300){xpid->x_output=-300;}
    return xpid->x_output;
}
int y_pid_calculator(y_pid_InitTypedef *ypid,float y_measure)
{
    ypid->y_error=ypid->y_setpoint-y_measure;
    ypid->y_intergreal=ypid->y_intergreal+(ypid->y_error*ypid->y_dt);
    ypid->y_derivative=(ypid->y_error-ypid->y_pre_error)/ypid->y_dt;
    ypid->y_output=(ypid->y_kp*ypid->y_error)+(ypid->y_ki*ypid->y_intergreal)+(ypid->y_kd*ypid->y_derivative);
    ypid->y_pre_error=ypid->y_error;
    //if(ypid->y_output>300){ypid->y_output=300;}if(ypid->y_output<-300){ypid->y_output=-300;}
    return ypid->y_output;
}
void RX_Interrupt(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        rx_buffer[rx_length]=rx_wait[0];
		rx_length++;
        if(rx_wait[0]=='\n')
        {
			kp=(float)((rx_buffer[0]-48)*100+(rx_buffer[2]-48)*10+(rx_buffer[3]-48))/(float)100;
			ki=(float)((rx_buffer[5]-48)*100+(rx_buffer[7]-48)*10+(rx_buffer[8]-48))/(float)100;
			kd=(float)((rx_buffer[10]-48)*100+(rx_buffer[12]-48)*10+(rx_buffer[13]-48))/(float)100;
            x_pid_InitStructure.x_kp=y_pid_InitStructure.y_kp=kp;
            x_pid_InitStructure.x_ki=y_pid_InitStructure.y_ki=ki;
            x_pid_InitStructure.x_kd=y_pid_InitStructure.y_kd=kd;
            rx_length=0;
            memset(rx_buffer,0,255);
			printf("ok");
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_wait, 1); //接收中断
    }
}
