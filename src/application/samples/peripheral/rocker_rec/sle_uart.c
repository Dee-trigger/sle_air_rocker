#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"

#include "sle_low_latency.h"

#include "securec.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"


#define SLE_UART_TASK_PRIO                  28
#define SLE_UART_TASK_DURATION_MS           2000
#define SLE_UART_BAUDRATE                   115200
#define SLE_UART_TRANSFER_SIZE              512
#define SLE_UART_BUS  1
#define UART_TXD_PIN 15
#define UART_RXD_PIN 16

static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };

static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

static void uart_init_pin(void)
{
    
        uapi_pin_set_mode(UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(UART_RXD_PIN, PIN_MODE_1);       
         
    
}

static void uart_init_config(void)
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
    uapi_uart_deinit(SLE_UART_BUS);
    uapi_uart_init(SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);

}
#define SLE_UART_SERVER_DELAY_COUNT         5

#define SLE_UART_TASK_STACK_SIZE            0x1200
#define SLE_ADV_HANDLE_DEFAULT              1
#define SLE_UART_SERVER_MSG_QUEUE_LEN       5
#define SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE  32
#define SLE_UART_SERVER_QUEUE_DELAY         0xFFFFFFFF
#define SLE_UART_SERVER_BUFF_MAX_SIZE       800

#define SLE_UART_SERVER_SEND_BUFF_MAX_LEN   40

unsigned long g_sle_uart_server_msgqueue_id;
#define SLE_UART_SERVER_LOG                 "[sle uart server]"
static void ssaps_server_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)//读取队列回调
{
    osal_printk("%s ssaps read request cbk callback server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
}

static void ssaps_server_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para,
    errcode_t status)//写回调
{
    osal_printk("%s ssaps write request callback cbk server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, write_cb_para->handle, status);
    if ((write_cb_para->length > 0) && write_cb_para->value) {
        osal_printk("\n sle uart recived data : %s\r\n", write_cb_para->value);
        uapi_uart_write(SLE_UART_BUS, (uint8_t *)write_cb_para->value, write_cb_para->length, 0);//收到什么发什么
        //osal_printk(write_cb_para->value);
    }
}


static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (sle_uart_client_is_connected()) {

    sle_uart_server_send_report_by_handle(buffer, length);//sle发送

    } else {
        osal_printk("%s sle client is not connected! \r\n", SLE_UART_SERVER_LOG);
    }
}


static void sle_uart_server_create_msgqueue(void)//创建消息队列
{
    if (osal_msg_queue_create("sle_uart_server_msgqueue", SLE_UART_SERVER_MSG_QUEUE_LEN, \
        (unsigned long *)&g_sle_uart_server_msgqueue_id, 0, SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        osal_printk("^%s sle_uart_server_create_msgqueue message queue create failed!\n", SLE_UART_SERVER_LOG);
    }
}

static void sle_uart_server_delete_msgqueue(void)//删除消息队列
{
    osal_msg_queue_delete(g_sle_uart_server_msgqueue_id);
}

static void sle_uart_server_write_msgqueue(uint8_t *buffer_addr, uint16_t buffer_size)//写队列
{
    osal_msg_queue_write_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                              (uint32_t)buffer_size, 0);
}

static int32_t sle_uart_server_receive_msgqueue(uint8_t *buffer_addr, uint32_t *buffer_size)//接收队列
{
    return osal_msg_queue_read_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                                    buffer_size, SLE_UART_SERVER_QUEUE_DELAY);
}
static void sle_uart_server_rx_buf_init(uint8_t *buffer_addr, uint32_t *buffer_size)//接收数据数组初始化
{
    *buffer_size = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    (void)memset_s(buffer_addr, *buffer_size, 0, *buffer_size);//缓冲检查
}

static void *sle_uart_server_task(const char *arg)//线程任务
{
    unused(arg);
    uint8_t rx_buf[SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE] = {0};
    uint32_t rx_length = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    uint8_t sle_connect_state[] = "sle_dis_connect";//状态初始化而已

    sle_uart_server_create_msgqueue();//创建消息队列
    sle_uart_server_register_msg(sle_uart_server_write_msgqueue);//注册消息队列
    sle_uart_server_init(ssaps_server_read_request_cbk, ssaps_server_write_request_cbk);



    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    uapi_uart_unregister_rx_callback(SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(SLE_UART_BUS, (1|2|4),1, sle_uart_server_read_int_handler);

    
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s Register uart callback fail.[%x]\r\n", SLE_UART_SERVER_LOG, ret);
        return NULL;
    }
    while (1) {
        sle_uart_server_rx_buf_init(rx_buf, &rx_length);
        sle_uart_server_receive_msgqueue(rx_buf, &rx_length);//接受消息进入队列
        if (strncmp((const char *)rx_buf, (const char *)sle_connect_state, sizeof(sle_connect_state)) == 0) {
            ret = sle_start_announce(SLE_ADV_HANDLE_DEFAULT);//开放扫描启动设备公
            if (ret != ERRCODE_SLE_SUCCESS) {
                osal_printk("%s sle_connect_state_changed_cbk,sle_start_announce fail :%02x\r\n",
                    SLE_UART_SERVER_LOG, ret);
            }
        }
        osal_msleep(SLE_UART_TASK_DURATION_MS);
    }
    sle_uart_server_delete_msgqueue();
    return NULL;
}


static void sle_uart_entry(void)//主任务
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();

    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task, 0, "SLEUartServerTask",
                                      SLE_UART_TASK_STACK_SIZE);

    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_UART_TASK_PRIO);
    }
    osal_kthread_unlock();
}

/* Run the sle_uart_entry. */
app_run(sle_uart_entry);