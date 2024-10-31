//client没用到串口，调试时用到了就保留了，可以不用管
#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "cmsis_os2.h"
#include "osal_debug.h"

#include "uart.h"
// #include "pm_clock.h"

#include "sle_low_latency.h"
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_uart_client.h"

#include "adc.h"
#include "adc_porting.h"

#include "gpio.h"
#include "hal_gpio.h"
#include "watchdog.h"

#define ADC_TASK_STACK_SIZE               0x1000
#define ADC_TASK_PRIO                     17

#define ADC_AUTO_SAMPLE_TEST_TIMES        2000

#define SLE_UART_TASK_STACK_SIZE            0x1000

#define SLE_UART_TASK_PRIO                  17

#define SLE_UART_TASK_DURATION_MS           2000
#define SLE_UART_BAUDRATE                   115200
#define SLE_UART_TRANSFER_SIZE              512
#define UART_TXD_PIN     15
#define UART_RXD_PIN     16
#define UART_BUS    1

#define KEY1_GPIO         10
#define KEY2_GPIO          9
#define KEY3_GPIO         11
#define TRIGGER_GPIO       5   
 //      gpio5 mode 在4
#define POS_UP_GPIO        1
#define POS_DOWN_GPIO      0
#define POS_LEFT_GPIO      3
#define POS_RIGHT_GPIO     2
#define POS_IN_GPIO        4    
//          GPIO4 MODE 2
#define Y_POS              0
#define X_POS              1

#define BUTTON_TASK_PRIO 17

#define BUTTON_TASK_STACK_SIZE 0x1000
//*************************************************************************test start
static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };//接受栈
static uint8_t sle_tx_buff[18] = { 0 };
static uint8_t test[]="haode1234";

static uart_buffer_config_t g_app_uart_buffer_config = {//接收配置
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

static void uart_init_pin(void)//uart引脚配置
{
    
        uapi_pin_set_mode(UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(UART_RXD_PIN, PIN_MODE_1);       
   
}

static void uart_init_config(void)//UART配置
{
    uart_attr_t attr = {
        .baud_rate = SLE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };

    uart_pin_config_t pin_config = {
        .tx_pin = UART_TXD_PIN,
        .rx_pin = UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(UART_BUS);
    uapi_uart_init(UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);

}

//******************************************************************************test end
static void gpio_callback_func(pin_t pin, uintptr_t param)
{
    UNUSED(pin);
    UNUSED(param);
    uint32_t pin_i=(uint32_t)pin;
   if (uapi_gpio_get_val(pin)==1)
   {  osal_mdelay(5);
        if (uapi_gpio_get_val(pin)==1)//消抖
        {
    osal_printk(" %d pressed.\r\n",pin_i);
    if(pin_i==POS_UP_GPIO)sle_tx_buff[9]='1';else sle_tx_buff[9]='0';
    if(pin_i==POS_DOWN_GPIO)sle_tx_buff[10]='1';else sle_tx_buff[10]='0';
    if(pin_i==POS_LEFT_GPIO)sle_tx_buff[11]='1';else sle_tx_buff[11]='0';
    if(pin_i==POS_RIGHT_GPIO)sle_tx_buff[12]='1';else sle_tx_buff[12]='0';
    if(pin_i==TRIGGER_GPIO)sle_tx_buff[13]='1';else sle_tx_buff[13]='0';
    if(pin_i==KEY1_GPIO)sle_tx_buff[14]='1';else sle_tx_buff[14]='0';
    if(pin_i==KEY2_GPIO)sle_tx_buff[15]='1';else sle_tx_buff[15]='0';
    if(pin_i==KEY3_GPIO)sle_tx_buff[16]='1';else sle_tx_buff[16]='0';
    if(pin_i==POS_UP_GPIO)sle_tx_buff[17]='1';else sle_tx_buff[17]='0';
    /* ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();
    sle_uart_send_param->data_len = 9;
    sle_uart_send_param->data = test;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
    for (int i =0; i <18; i++) {  //清零
         printf("%c",sle_tx_buff[i]);
         sle_tx_buff[i] = '0';   }*/
        }
   }
    

 } 
    

void gpio_set_for_key(uint32_t key)
{
    uapi_pin_set_mode(key, HAL_PIO_FUNC_GPIO);//
    uapi_gpio_set_dir(key, 0);//输入mode
    uapi_pin_set_pull(key, 1);//PIN_PULL_TYPE_DOWN      = 1,
    uapi_gpio_register_isr_func(key, (0x00000001 | 0x00000002), gpio_callback_func);
}
void gpio_init(uint8_t no)
{
    unused(no);
    gpio_set_for_key(KEY1_GPIO);
    gpio_set_for_key(KEY2_GPIO);
    gpio_set_for_key(KEY3_GPIO);
    gpio_set_for_key(POS_UP_GPIO);
    gpio_set_for_key(POS_DOWN_GPIO );
    gpio_set_for_key(POS_LEFT_GPIO);
    gpio_set_for_key(POS_RIGHT_GPIO);
//gpio4 and gpio5 的gpio mode跟其他的不同，需要单独设置 
    uapi_pin_set_mode(TRIGGER_GPIO, 4);//gpio5 mode 在4
    uapi_gpio_set_dir(TRIGGER_GPIO, 0);//输入mode
    uapi_pin_set_pull(TRIGGER_GPIO, 1);
    uapi_gpio_register_isr_func(TRIGGER_GPIO, (0x00000001 | 0x00000002), gpio_callback_func);

    uapi_pin_set_mode(POS_IN_GPIO, 2);// GPIO4 MODE 2
    uapi_gpio_set_dir(POS_IN_GPIO, 0);//输入mode
    uapi_pin_set_pull(POS_IN_GPIO, 1);
    uapi_gpio_register_isr_func(POS_IN_GPIO, (0x00000001 | 0x00000002), gpio_callback_func);
    


}
void adc_callback(uint8_t ch, uint32_t *buffer, uint32_t length, bool *next)//adc回调函数
{    uint8_t  x_one_r='0', x_ten_r='0', x_hundred_r='0', x_thousand_r='0',
              y_one_r='0', y_ten_r='0', y_hundred_r='0', y_thousand_r='0';
    uint32_t  x_one_i=0, x_ten_i=0, x_hundred_i=0, x_thousand_i=0,
              y_one_i=0, y_ten_i=0, y_hundred_i=0, y_thousand_i=0;
     UNUSED(next);

     
    for (uint32_t i = 0; i < length; i++) {
        if(abs(buffer[i])>50){
            osal_msleep(5);
            if(abs(buffer[i])>50)
                printf("channel: %d, voltage: %dmv\r\n", ch, buffer[i]);
            else return ;
        }
    } 
    if(ch=='1'){
     x_one_i=(*buffer)%10;
     x_ten_i=((*buffer)/10)%10;
     x_hundred_i=((*buffer)/100)%10;
     x_thousand_i=(*buffer)/1000;

     x_one_r='0' +  x_one_i;
     x_ten_r='0' + x_ten_i ;
     x_hundred_r='0' +  x_hundred_i;
     x_thousand_r='0' +  x_thousand_i;
     printf("teat_channel: %d, voltage: %d %d %d %dmv\r\n", ch,x_thousand_i, x_hundred_i,x_ten_i,x_one_i);
    }
    else
    {
     y_one_i=(*buffer)%10;
     y_ten_i=((*buffer)/10)%10;
     y_hundred_i=((*buffer)/100)%10;
     y_thousand_i=(*buffer)/1000;

     y_one_r='0' +  y_one_i;
     y_ten_r='0' + y_ten_i ;
     y_hundred_r='0' +  y_hundred_i;
     y_thousand_r='0' +  y_thousand_i;
    }

    sle_tx_buff[0]='1';

    sle_tx_buff[1]=x_thousand_r;
    sle_tx_buff[2]=x_hundred_r;
    sle_tx_buff[3]=x_ten_r;
    sle_tx_buff[4]=x_one_r;

    sle_tx_buff[5]=y_thousand_r;
    sle_tx_buff[6]=y_hundred_r;
    sle_tx_buff[7]=y_ten_r;
    sle_tx_buff[8]=y_one_r;

    /* ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();
    sle_uart_send_param->data_len = 9;
    sle_uart_send_param->data = test;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param); */
    



}

void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)//sle通知回调
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(UART_BUS, (uint8_t *)(data->data), data->data_len, 0);//写UART
}

void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)//sle指令回调
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(UART_BUS, (uint8_t *)(data->data), data->data_len, 0);//写UART
}

static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();
    sle_uart_send_param->data_len = length;
    sle_uart_send_param->data = (uint8_t *)buffer;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
}

static void *sle_client_task(const char *arg)//sle线程任务
{
    unused(arg);
    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();
     //uint16_t g_sle_uart_conn_id;
    uapi_uart_unregister_rx_callback(UART_BUS);//注册UART1的回调函数

    errcode_t ret = uapi_uart_register_rx_callback(UART_BUS,(1|2|4),1, sle_uart_client_read_int_handler);

    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);//sle 回调函数注册兼调用
    osal_mdelay(2000);
    
    if (ret != ERRCODE_SUCC) {
        osal_printk("Register uart callback fail.");
        return NULL;
    }
   while(1){
     osal_printk("sle start send");
    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();
    sle_uart_send_param->data_len = 9;
    sle_uart_send_param->data = test;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
    osal_printk("sle end send");
   }
    return NULL;
}
static void *adc_task(const char *arg)//adc线程任务
{
    UNUSED(arg);
    osal_printk("start adc sample test");
    uapi_adc_init(ADC_CLOCK_500KHZ);
    uapi_adc_power_en(AFE_SCAN_MODE_MAX_NUM, true);
    adc_scan_config_t config = {
        .type = 0,
        .freq = 1,
        
    };
     osal_mdelay(2000);
    while (1)
    {
        uapi_adc_auto_scan_ch_enable(X_POS, config, adc_callback);
        uapi_adc_auto_scan_ch_disable(X_POS);
        uapi_adc_auto_scan_ch_enable(Y_POS, config, adc_callback);
        uapi_adc_auto_scan_ch_disable(Y_POS);
        //osal_msleep(ADC_AUTO_SAMPLE_TEST_TIMES);
    }
    
    return NULL;
}
static void *button_task(const char *arg)//gpio 任务
{   uint8_t a=0;//凑数的，没啥用
    unused(arg);
    gpio_init(a);
    osal_mdelay(2000);


    

    while (1) {
        uapi_watchdog_kick(); // 没事干就喂狗
       
    }
    return NULL;
}
static void sle_entry(void)//sle入口函数
{
    osal_task *sle_task_handle = NULL;
    osal_kthread_lock();


    sle_task_handle = osal_kthread_create((osal_kthread_handler)sle_client_task, 0, "SLEUartDongleTask",
                                      SLE_UART_TASK_STACK_SIZE);
   
    if (sle_task_handle != NULL) {
        osal_kthread_set_priority(sle_task_handle, SLE_UART_TASK_PRIO);
    }
    osal_kthread_unlock();
    
    
    
}

static void adc_entry(void)//adc入口函数
{
    osal_task *adc_task_handle = NULL;
    osal_kthread_lock();


adc_task_handle = osal_kthread_create((osal_kthread_handler)adc_task, 0, "ADCTask",
                                      ADC_TASK_STACK_SIZE);

    if (adc_task_handle != NULL) {
        osal_kthread_set_priority(adc_task_handle, ADC_TASK_PRIO);
    }
    osal_kthread_unlock();



}

static void button_entry(void)//按键入口
{
    uint32_t ret;
    osal_task *taskid;
    // 创建任务调度
    osal_kthread_lock();
    // 创建任务1
    taskid = osal_kthread_create((osal_kthread_handler)button_task, NULL, "led_task", BUTTON_TASK_STACK_SIZE);
    ret = osal_kthread_set_priority(taskid, BUTTON_TASK_PRIO );
    if (ret != OSAL_SUCCESS) {
        printf("create task1 failed .\n");
    }
    osal_kthread_unlock();
}



static void main_entry(void)
{  sle_entry();

   adc_entry();
   
   button_entry();
}
/* Run the sle_uart_entry. */
app_run(main_entry);