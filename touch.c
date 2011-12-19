#include <stdbool.h>
#include "stm32f10x.h"

#include "board.h"
#include "touch.h"

#include <rtthread.h>
#include <rtgui/event.h>
#include <rtgui/kbddef.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>

/*
MISO PA6
MOSI PA7
CLK  PA5
CS   PC4
*/

#define CS_PORT          GPIOB
#define CS_PIN           GPIO_Pin_7
#define CS_RCC           RCC_APB2Periph_GPIOB

#define TC_INT_PORT      GPIOB
#define TC_INT_PIN       GPIO_Pin_6
#define TC_INT_RCC       RCC_APB2Periph_GPIOB
#define TC_EXTI_PORT     GPIO_PortSourceGPIOB
#define TC_EXTI_PIN      GPIO_PinSource6
#define NVIC_CHN         EXTI9_5_IRQn
#define EXTI_LINE        EXTI_Line6

#define SPI_CHN          SPI1
#define SPI_PORT         GPIOA
#define SPI_PIN          GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
#define SPI_RCC          RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1

#define   CS_0()         GPIO_ResetBits(CS_PORT,CS_PIN)
#define   CS_1()         GPIO_SetBits(CS_PORT,CS_PIN)

#define TOUCH_SCAN_INTERVAL (RT_TICK_PER_SECOND/50)
#define MOUSE_MOVE_THRESHOLD      8
#define MOUSE_MOVE_LIMIT        100

/*
7  6 - 4  3      2     1-0
s  A2-A0 MODE SER/DFR PD1-PD0
*/
#define TOUCH_MSR_Y  0x90   //读X轴坐标指令 addr:1
#define TOUCH_MSR_X  0xD0   //读Y轴坐标指令 addr:3

struct touch_point
{
    rt_uint16_t x, y;
};

struct rtgui_touch_device
{
    struct rt_device parent;

    rt_timer_t poll_timer;
    rt_uint16_t x, y;

    rt_bool_t calibrating;
    rt_touch_calibration_func_t calibration_func;

    rt_uint16_t min_x, max_x;
    rt_uint16_t min_y, max_y;
};
static struct rtgui_touch_device *touch = RT_NULL;

extern unsigned char SPI_WriteByte(unsigned char data);
rt_inline void EXTI_Enable(rt_uint32_t enable);
static rt_err_t rtgui_touch_control (rt_device_t dev, rt_uint8_t cmd, void *args);
static void WriteDataTo7843(unsigned char num);
void touch_timeout(void* parameter);

struct rt_semaphore spi1_lock;

static void NVIC_enable(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = NVIC_CHN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

rt_inline void EXTI_Enable(rt_uint32_t enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//Falling下降沿 Rising上升

    if (enable)
    {
        /* enable */
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    }
    else
    {
        /* disable */
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    }

    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(EXTI_LINE);
}

static void EXTI_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(TC_INT_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = TC_INT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(TC_INT_PORT,&GPIO_InitStructure);

    GPIO_EXTILineConfig(TC_EXTI_PORT, TC_EXTI_PIN);

    /* Configure  EXTI  */
    EXTI_Enable(1);
}

/* RT-Thread Device Interface */
static rt_err_t rtgui_touch_init (rt_device_t dev)
{
    NVIC_enable();
    EXTI_Configuration();

    /* PC4 touch CS */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(CS_RCC, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin   = CS_PIN;
        GPIO_Init(CS_PORT,&GPIO_InitStructure);
        CS_1();
    }

    CS_0();
    WriteDataTo7843( 1<<7 ); /* 打开中断 */
    CS_1();

    return RT_EOK;
}

void EXTI9_5_IRQHandler(void)
{
    /* disable interrupt */
    EXTI_Enable(0);

    /* start timer */
    rt_timer_start(touch->poll_timer);

    EXTI_ClearITPendingBit(EXTI_LINE);
}

void rtgui_touch_hw_init(void)
{
    /* SPI_CHN config */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        SPI_InitTypeDef SPI_InitStructure;

        /* Enable SPI_CHN Periph clock */
        RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);

        /* Configure SPI_CHN pins: PA5-SCK, PA6-MISO and PA7-MOSI */
        GPIO_InitStructure.GPIO_Pin = SPI_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(SPI_PORT, &GPIO_InitStructure);

        /*------------------------ SPI_CHN configuration ------------------------*/
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;/* 72M/64=1.125M */
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;

        SPI_I2S_DeInit(SPI_CHN);
        SPI_Init(SPI_CHN, &SPI_InitStructure);

        /* Enable SPI_MASTER */
        SPI_Cmd(SPI_CHN, ENABLE);
        SPI_CalculateCRC(SPI_CHN, DISABLE);

        if (rt_sem_init(&spi1_lock, "spi1lock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("init spi1 lock semaphore failed\n");
        }
    } /* SPI_CHN config */

    touch = (struct rtgui_touch_device*)rt_malloc (sizeof(struct rtgui_touch_device));
    if (touch == RT_NULL) return; /* no memory yet */

    /* clear device structure */
    rt_memset(&(touch->parent), 0, sizeof(struct rt_device));
    touch->calibrating = false;

    /* init device structure */
    touch->parent.type = RT_Device_Class_Unknown;
    touch->parent.init = rtgui_touch_init;
    touch->parent.control = rtgui_touch_control;
    touch->parent.user_data = RT_NULL;

    touch->poll_timer = rt_timer_create("touch", touch_timeout, RT_NULL,
                                        TOUCH_SCAN_INTERVAL, RT_TIMER_FLAG_PERIODIC);

    rt_device_register(&(touch->parent), "touch", RT_DEVICE_FLAG_RDWR);
}

void rt_hw_spi1_baud_rate(uint16_t SPI_BaudRatePrescaler)
{
    SPI_CHN->CR1 &= ~SPI_BaudRatePrescaler_256;
    SPI_CHN->CR1 |= SPI_BaudRatePrescaler;
}

uint8_t SPI_WriteByte(unsigned char data)
{
    //Wait until the transmit buffer is empty
    while (SPI_I2S_GetFlagStatus(SPI_CHN, SPI_I2S_FLAG_TXE) == RESET);
    // Send the byte
    SPI_I2S_SendData(SPI_CHN, data);

    //Wait until a data is received
    while (SPI_I2S_GetFlagStatus(SPI_CHN, SPI_I2S_FLAG_RXNE) == RESET);
    // Get the received data
    data = SPI_I2S_ReceiveData(SPI_CHN);

    // Return the shifted data
    return data;
}

//SPI写数据
static void WriteDataTo7843(unsigned char num)
{
    SPI_WriteByte(num);
}

#define X_WIDTH 240
#define Y_WIDTH 320

static struct touch_point rtgui_touch_calculate()
{
    struct touch_point tmppos[10] = {0};
    unsigned int i;

    rt_sem_take(&spi1_lock, RT_WAITING_FOREVER);
    /* SPI_CHN configure */
    rt_hw_spi1_baud_rate(SPI_BaudRatePrescaler_256);

    for(i=0; i<10; i++)
    {
        CS_0();

        WriteDataTo7843(TOUCH_MSR_X);
        tmppos[i].x = (SPI_WriteByte(0x00) & 0x7F) << 5;
        tmppos[i].x |= (SPI_WriteByte(TOUCH_MSR_Y) >> 3) & 0x1F;

        tmppos[i].y = (SPI_WriteByte(0x00) & 0x7F) << 5;
        tmppos[i].y |= (SPI_WriteByte(0x00) >> 3) & 0x1F;

        WriteDataTo7843( 1<<7 );
        CS_1();
    }

    //去最高值与最低值,再取平均值
    {
        rt_uint32_t min_x = 0xFFFF,min_y = 0xFFFF;
        rt_uint32_t max_x = 0,max_y = 0;
        rt_uint32_t total_x = 0;
        rt_uint32_t total_y = 0;
        unsigned int i;

        for(i=0;i<10;i++)
        {
            if( tmppos[i].x < min_x )
            {
                min_x = tmppos[i].x;
            }
            if( tmppos[i].x > max_x )
            {
                max_x = tmppos[i].x;
            }
            total_x += tmppos[i].x;

            if( tmppos[i].y < min_y )
            {
                min_y = tmppos[i].y;
            }
            if( tmppos[i].y > max_y )
            {
                max_y = tmppos[i].y;
            }
            total_y += tmppos[i].y;
        }
        total_x = total_x - min_x - max_x;
        total_y = total_y - min_y - max_y;
        /*use tmppos[0] as the final result */
        tmppos[0].x = total_x / 8;
        tmppos[0].y = total_y / 8;
    }

    rt_sem_release(&spi1_lock);

    if (touch->calibrating != RT_TRUE)
    {
        /*rt_kprintf("raw value: %d, %d\n\r", tmppos[0].x, tmppos[0].y);*/
        if (touch->max_x > touch->min_x)
        {
            tmppos[0].x = (tmppos[0].x - touch->min_x) * X_WIDTH/(touch->max_x - touch->min_x);
        }
        else if (touch->max_x < touch->min_x)
        {
            tmppos[0].x = (touch->min_x - tmppos[0].x) * X_WIDTH/(touch->min_x - touch->max_x);
        }

        if (touch->max_y > touch->min_y)
        {
            tmppos[0].y = (tmppos[0].y - touch->min_y) * Y_WIDTH /(touch->max_y - touch->min_y);
        }
        else if (touch->max_y < touch->min_y)
        {
            tmppos[0].y = (touch->min_y - tmppos[0].y) * Y_WIDTH /(touch->min_y - touch->max_y);
        }

	// no invalid value
	if (tmppos[0].x & 0x8000)
		tmppos[0].x = 0;
	else if (tmppos[0].x > X_WIDTH)
		tmppos[0].x = X_WIDTH - 1;

	if (tmppos[0].y & 0x8000)
		tmppos[0].y = 0;
	else if (tmppos[0].y > Y_WIDTH)
		tmppos[0].y = Y_WIDTH - 1;
    }

    return tmppos[0];
}

void touch_timeout(void* parameter)
{
        static unsigned int touch_down = 0;
        struct rtgui_event_mouse emouse;
        static struct touch_point touch_previous;

        /* touch time is too short and we lost the position already. */
        if ((!touch_down) && GPIO_ReadInputDataBit(TC_INT_PORT, TC_INT_PIN) != 0)
            return;

        if (touch_down && GPIO_ReadInputDataBit(TC_INT_PORT, TC_INT_PIN) != 0)
        {
                int tmer = TOUCH_SCAN_INTERVAL;
                EXTI_Enable(1);
                emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
                emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_UP);

                /* use old value */
                emouse.x = touch->x;
                emouse.y = touch->y;

                /*rt_kprintf("touch up: (%d, %d)\n\r", emouse.x, emouse.y);*/
                touch_down = 0;

                if ((touch->calibrating == RT_TRUE) && (touch->calibration_func != RT_NULL))
                {
                        /* callback function */
                        touch->calibration_func(emouse.x, emouse.y);
                }
                /* stop timer */
                rt_timer_control(touch->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
                rt_timer_stop(touch->poll_timer);
        }
        else
        {
                if(touch_down == 0)
                {
                        int tmer = TOUCH_SCAN_INTERVAL;

                        touch_previous = rtgui_touch_calculate();
                        touch->x = touch_previous.x;
                        touch->y = touch_previous.y;

                        /* send mouse event */
                        emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
                        emouse.parent.sender = RT_NULL;

                        emouse.x = touch->x;
                        emouse.y = touch->y;

                        /* init mouse button */
                        emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_DOWN);

                        /*rt_kprintf("touch down: (%d, %d)\n\r", emouse.x, emouse.y);*/
                        touch_down = 1;
                        rt_timer_control(touch->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
                }
                else
                {
                    struct touch_point touch_new = rtgui_touch_calculate();
                        //判断移动距离是否小于MOUSE_MOVE_THRESHOLD,减少误动作.
                        if(  (touch_previous.x < touch_new.x + MOUSE_MOVE_THRESHOLD)
                          && (touch_previous.x > touch_new.x - MOUSE_MOVE_THRESHOLD)
                          && (touch_previous.y < touch_new.y + MOUSE_MOVE_THRESHOLD)
                          && (touch_previous.y > touch_new.y - MOUSE_MOVE_THRESHOLD))
                        {
                                return;
                        }

			// man cannot move too much between
			if ((touch_new.x < touch_previous.x - MOUSE_MOVE_LIMIT)
			 || (touch_previous.x + MOUSE_MOVE_LIMIT < touch_new.x)
			 || (touch_new.y < touch_previous.y - MOUSE_MOVE_LIMIT)
			 || (touch_previous.y + MOUSE_MOVE_LIMIT < touch_new.y))
			{
				return;
			}

                        touch_previous = touch_new;
			touch->x = touch_new.x;
			touch->y = touch_new.y;

                        /* send mouse event */
                        emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
                        emouse.parent.sender = RT_NULL;

                        emouse.x = touch_new.x;
                        emouse.y = touch_new.y;

                        /* init mouse button */
                        emouse.button = (RTGUI_MOUSE_BUTTON_RIGHT |RTGUI_MOUSE_BUTTON_DOWN);
                        /*rt_kprintf("touch motion: (%d, %d)\n\r", emouse.x, emouse.y);*/
                }
        }

    /* send event to server */
    if (touch->calibrating != RT_TRUE)
        rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));
}

static rt_err_t rtgui_touch_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case RT_TOUCH_CALIBRATION:
        touch->calibrating = RT_TRUE;
        touch->calibration_func = (rt_touch_calibration_func_t)args;
        break;

    case RT_TOUCH_NORMAL:
        touch->calibrating = RT_FALSE;
        break;

    case RT_TOUCH_CALIBRATION_DATA:
    {
        struct calibration_data* data;

        data = (struct calibration_data*) args;

        //update
        touch->min_x = data->min_x;
        touch->max_x = data->max_x;
        touch->min_y = data->min_y;
        touch->max_y = data->max_y;
    }
    break;
    }

    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void touch_t( rt_uint16_t x , rt_uint16_t y )
{
    struct rtgui_event_mouse emouse ;
    emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
    emouse.parent.sender = RT_NULL;

    emouse.x = x ;
    emouse.y = y ;
    /* init mouse button */
    emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_DOWN );
    rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));

    rt_thread_delay(2) ;
    emouse.button = (RTGUI_MOUSE_BUTTON_LEFT |RTGUI_MOUSE_BUTTON_UP );
    rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));
}

FINSH_FUNCTION_EXPORT(touch_t, x & y ) ;
#endif
