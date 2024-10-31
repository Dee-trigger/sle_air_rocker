#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

#include "pinctrl.h"

#include "i2c.h"
#include "soc_osal.h"
#include "app_init.h"
#if defined(CONFIG_I2C_SUPPORT_DMA) && (CONFIG_I2C_SUPPORT_DMA == 1)
#include "dma.h"
#endif

#define I2C_MASTER_ADDR                   0x0
#define I2C_SLAVE_ADDR                    0xD0
#define I2C_SET_BAUDRATE                  50000
#define I2C_TASK_DURATION_MS              500
#if defined(CONFIG_I2C_SUPPORT_INT) && (CONFIG_I2C_SUPPORT_INT == 1)
#define I2C_INT_TRANSFER_DELAY_MS         800
#endif

#define I2C_TASK_PRIO                     24
#define I2C_TASK_STACK_SIZE               0x1000
int AX, AY, AZ, GX, GY, GZ;
static void app_i2c_init_pin(void)
{
    /* I2C pinmux. */
    uapi_pin_set_mode(16, 2);
    uapi_pin_set_mode(15, 2);
}
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{   i2c_data_t RegAddress_s = { 0 };
    RegAddress_s.send_buf = &RegAddress;
    RegAddress_s.send_len = 8;

    i2c_data_t Data_s = { 0 };
    Data_s.send_buf = &Data;
    Data_s.send_len = 8;

	
	uapi_i2c_master_write(1, I2C_SLAVE_ADDR, &RegAddress_s);
	osal_msleep(50);
	uapi_i2c_master_write(1, I2C_SLAVE_ADDR, &Data_s);
	osal_msleep(50);
	
	
}
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{   i2c_data_t RegAddress_s = { 0 };
    RegAddress_s.send_buf = &RegAddress;
    RegAddress_s.send_len = 8;
    
    uint8_t rx_buff[8] = { 0 };
    i2c_data_t Data = { 0 };
    Data.receive_buf = rx_buff;
    Data.receive_len = 8;

	
	uapi_i2c_master_write(1, I2C_SLAVE_ADDR, &RegAddress_s);
	osal_msleep(50);
    uapi_i2c_master_read(1, I2C_SLAVE_ADDR, &Data);
	osal_msleep(50);
	
	return *Data.receive_buf;
}
void MPU6050_GetData(int *AccX, int *AccY, int *AccZ, int *GyroX, int *GyroY, int *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}

static void *i2c_master_task(const char *arg)//线程
{
    unused(arg);
    //i2c_data_t data = { 0 };

    uint32_t baudrate = I2C_SET_BAUDRATE;//数据率
    uint8_t hscode = I2C_MASTER_ADDR;//主地址
    //uint16_t dev_addr = I2C_SLAVE_ADDR;//从地址

#if defined(CONFIG_I2C_SUPPORT_DMA) && (CONFIG_I2C_SUPPORT_DMA == 1)
    uapi_dma_init();
    uapi_dma_open();
#endif  /* CONFIG_I2C_SUPPORT_DMA */

    /* I2C master init config. */
    app_i2c_init_pin();
    uapi_i2c_master_init(1, baudrate, hscode);//(i2c_id,波特率，主地址)

#if defined(CONFIG_I2C_SUPPORT_INT) && (CONFIG_I2C_SUPPORT_INT == 1)
    uapi_i2c_set_irq_mode(CONFIG_I2C_MASTER_BUS_ID, 1);
#endif  /* CONFIG_I2C_SUPPORT_INT */

    /* I2C data config. */
   // uint8_t tx_buff[CONFIG_I2C_TRANSFER_LEN] = { 0 };
    //for (uint32_t loop = 0; loop < CONFIG_I2C_TRANSFER_LEN; loop++) {
    //    tx_buff[loop] = (loop & 0xFF);
    //}

    //uint8_t rx_buff[CONFIG_I2C_TRANSFER_LEN] = { 0 };
    //data.send_buf = tx_buff;
    //data.send_len = CONFIG_I2C_TRANSFER_LEN;
    //data.receive_buf = rx_buff;
    //data.receive_len = CONFIG_I2C_TRANSFER_LEN;
    osal_printk("i2c  start!\r\n");
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
    osal_printk("i2c master init ok!\r\n");
    while (1) {
        osal_msleep(I2C_TASK_DURATION_MS);
        //osal_printk("i2c master send start!\r\n");

        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        osal_printk("AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d\r\n", AX,AY,AZ,GX,GY,GZ);


#if defined(CONFIG_I2C_SUPPORT_INT) && (CONFIG_I2C_SUPPORT_INT == 1)
        osal_msleep(I2C_INT_TRANSFER_DELAY_MS);
#endif
   
        
    }

    return NULL;
}

static void i2c_master_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)i2c_master_task, 0, "I2cMasterTask", I2C_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, I2C_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

/* Run the i2c_master_entry. */
app_run(i2c_master_entry);