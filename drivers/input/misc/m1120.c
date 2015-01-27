/*************************************************************
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : m1120.c
 ** Description : 
 ** Date        : 2014-05-08 22:00
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include "m1120.h"

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#define M1120_STATUS_SIGNAL

#include <linux/proc_fs.h> 
static struct proc_dir_entry *rotordir = NULL;
static char* rotor_node_name = "cam_status";

#ifdef M1120_STATUS_SIGNAL
#include <linux/fs.h>
static struct fasync_struct *async_queue = NULL;
#endif

/* ********************************************************* */
/* customer config */ 
/* ********************************************************* */
#define M1120_DBG_ENABLE					// for debugging
#define M1120_DETECTION_MODE				M1120_DETECTION_MODE_INTERRUPT
#define M1120_INTERRUPT_TYPE				M1120_VAL_INTSRS_INTTYPE_BESIDE
#define M1120_SENSITIVITY_TYPE				M1120_VAL_INTSRS_SRS_0_017mT
#define M1120_PERSISTENCE_COUNT				M1120_VAL_PERSINT_COUNT(2)
#define M1120_OPERATION_FREQUENCY			M1120_VAL_OPF_FREQ_10HZ
#define M1120_OPERATION_RESOLUTION			M1120_VAL_OPF_BIT_10
#define M1120_DETECT_RANGE_HIGH				(10)
#define M1120_DETECT_RANGE_LOW				(-10)
#define M1120_RESULT_STATUS_A				(0x01)	// result status A
#define M1120_RESULT_STATUS_B				(0x02)	// result status B
#define M1120_EVENT_TYPE					EV_KEY
#define M1120_EVENT_CODE					KEY_F2
#define M1120_EVENT_DATA_CAPABILITY_MIN		(-32768)
#define M1120_EVENT_DATA_CAPABILITY_MAX		(32767)

/* ********************************************************* */
/* debug macro */
/* ********************************************************* */
#ifdef M1120_DBG_ENABLE
#define dbg(fmt, args...)  printk("[M1120-DBG] %s(%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define dbgn(fmt, args...)  printk(fmt, ##args)
#else
#define dbg(fmt, args...)   
#define dbgn(fmt, args...)  
#endif // M1120_DBG_ENABLE
#define dbg_func_in()       dbg("[M1120-DBG-F.IN] %s\n", __func__)
#define dbg_func_out()      dbg("[M1120-DBG-F.OUT] %s\n", __func__)
#define dbg_line()          dbg("[LINE] %d(%s)\n", __LINE__, __func__)
/* ********************************************************* */
/* error display macro */
/* ********************************************************* */
#define mxerr(pdev, fmt, args...)			\
    dev_err(pdev, "[M1120-ERR] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args) 
#define mxinfo(pdev, fmt, args...)			\
    dev_info(pdev, "[M1120-INFO] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args) 
/* ********************************************************* */
/* static variable */
/* ********************************************************* */
static m1120_data_t *p_m1120_data = NULL;
static int g_is_back = -1;

// for DTS
static struct of_device_id dhall_device_id[] = {
    {.compatible = "magna,dhall",},
    {},
};

/* ********************************************************* */
/* function protyps */
/* ********************************************************* */
/* i2c interface */
static int	m1120_i2c_read(struct i2c_client *client, u8 reg, u8* rdata, u8 len);
static int	m1120_i2c_get_reg(struct i2c_client *client, u8 reg, u8* rdata);
static int	m1120_i2c_write(struct i2c_client *client, u8 reg, u8* wdata, u8 len);
static int	m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata);
/* vdd / vid power control */
static int m1120_set_power(struct device *dev, bool on);
/* scheduled work */
static void m1120_work_func(struct work_struct *work);
/* interrupt handler */
static irqreturn_t m1120_irq_handler(int irq, void *dev_id);
/* configuring or getting configured status */
static void m1120_get_reg(struct device *dev, int* regdata);
static void m1120_set_reg(struct device *dev, int* regdata);
static int	m1120_get_enable(struct device *dev);
static void	m1120_set_enable(struct device *dev, int enable);
static int	m1120_get_delay(struct device *dev);
static void	m1120_set_delay(struct device *dev, int delay);
static int	m1120_get_debug(struct device *dev);
static void	m1120_set_debug(struct device *dev, int debug);
static int	m1120_clear_interrupt(struct device *dev);
static int	m1120_update_interrupt_threshold(struct device *dev);
static int	m1120_set_operation_mode(struct device *dev, int mode);
static int	m1120_set_detection_mode(struct device *dev, u8 mode);
static int	m1120_init_device(struct device *dev);
static int	m1120_reset_device(struct device *dev);
static int	m1120_get_calibrated_data(struct device *dev, int* data);
static int	m1120_measure(m1120_data_t *p_data, short *raw);
/* ********************************************************* */

/* extern func for camera */
int getCameraStatusByDhall(void)
{
    return g_is_back;  // 1: back   0: front   -1:invalid
}

/* ********************************************************* */
/* functions for i2c interface */
/* ********************************************************* */
#define M1120_I2C_BUF_SIZE					(17)
static int m1120_i2c_read(struct i2c_client* client, u8 reg, u8* rdata, u8 len)
{
    int rc;

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &reg,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = rdata,
        },
    };

    if ( client == NULL ) {
        mxerr(&client->dev, "client is NULL");
        return -ENODEV;
    }

    rc = i2c_transfer(client->adapter, msg, 2);
    if(rc<0) {
        mxerr(&client->dev, "i2c_transfer was failed(%d)", rc);
        return rc;
    }

    return 0;
}

static int	m1120_i2c_get_reg(struct i2c_client *client, u8 reg, u8* rdata)
{
    return m1120_i2c_read(client, reg, rdata, 1);
}

static int m1120_i2c_write(struct i2c_client* client, u8 reg, u8* wdata, u8 len)
{
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8  buf[M1120_I2C_BUF_SIZE];
    int rc;
    int i;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = len+1,
            .buf = buf,
        },
    };

    if ( client == NULL ) {
        printk("[ERROR] %s : i2c client is NULL.\n", __func__);
        return -ENODEV;
    }

    buf[0] = reg;
    if (len > M1120_I2C_BUF_SIZE) {
        mxerr(&client->dev, "i2c buffer size must be less than %d", M1120_I2C_BUF_SIZE);
        return -EIO;
    }
    for( i=0 ; i<len; i++ ) buf[i+1] = wdata[i];

    rc = i2c_transfer(client->adapter, msg, 1);
    if(rc< 0) {
        mxerr(&client->dev, "i2c_transfer was failed (%d)", rc);
        return rc;
    }

    if(len==1) {
        switch(reg){
            case M1120_REG_PERSINT:
                p_data->reg.map.persint = wdata[0];
                break;
            case M1120_REG_INTSRS:
                p_data->reg.map.intsrs = wdata[0];
                break;
            case M1120_REG_LTHL:
                p_data->reg.map.lthl = wdata[0];
                break;
            case M1120_REG_LTHH:
                p_data->reg.map.lthh = wdata[0];
                break;
            case M1120_REG_HTHL:
                p_data->reg.map.hthl = wdata[0];
                break;
            case M1120_REG_HTHH:
                p_data->reg.map.hthh = wdata[0];
                break;
            case M1120_REG_I2CDIS:
                p_data->reg.map.i2cdis = wdata[0];
                break;
            case M1120_REG_SRST:
                p_data->reg.map.srst = wdata[0];
                break;
            case M1120_REG_OPF:
                p_data->reg.map.opf = wdata[0];
                break;
        }
    }

    return 0;
}

static int m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata)
{
    //printk("%s reg:0x%x  val:0x%x \n", __FUNCTION__, reg, wdata);
    return m1120_i2c_write(client, reg, &wdata, sizeof(wdata));
}

/* vdd / vid power control */
static int m1120_set_power(struct device *dev, bool on)
{
    int rc;	
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    if(on) 
    {
        if (regulator_count_voltages(p_data->power_vdd) > 0) 
        {
            rc = regulator_set_voltage(p_data->power_vdd, 2700000, 2850000);
            if (rc) {
                dev_err(&p_data->client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
                return rc;
            }
        }
        if (regulator_count_voltages(p_data->power_vi2c) > 0) {
            rc = regulator_set_voltage(p_data->power_vi2c, 1800000, 1800000);
            if (rc) {
                dev_err(&p_data->client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
                return rc;
            }
        }
        /***enable the 2v8 power*****/
        rc = regulator_enable(p_data->power_vdd);
        if (rc) {
            dev_err(&p_data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
            return rc;
        }
        /***should enable the 1v8 power*****/
        msleep(1);
        rc = regulator_enable(p_data->power_vi2c);
        if (rc) {
            dev_err(&p_data->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
            regulator_disable(p_data->power_vdd);
            return rc;
        }
    } 
    else 
    {
        rc = regulator_disable(p_data->power_vi2c);
        if (rc) {
            dev_err(&p_data->client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
            regulator_enable(p_data->power_vdd);
            return rc;
        }

        msleep(1);

        rc = regulator_disable(p_data->power_vdd);
        if (rc) {
            dev_err(&p_data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
            return rc;
        }
    }

    return 0;
}
/* ********************************************************* */

/* ********************************************************* */
/* functions for scheduling */
/* ********************************************************* */
static void m1120_work_func(struct work_struct *work)
{
    m1120_data_t* p_data = container_of((struct delayed_work *)work, m1120_data_t, work);
    unsigned long delay = msecs_to_jiffies(m1120_get_delay(&p_data->client->dev));
    short raw = -1024;
    int err = 0;

    printk("%s enter\n",__func__);

    msleep(30);
    err = m1120_measure(p_data, &raw);
    if (err != 0)
    {
        printk("%s read adc err !!!\n",__func__);
        msleep(10);
        if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT)
        {
            enable_irq(p_m1120_data->irq);  
        }
        return;
    }

    g_is_back = !g_is_back;

    input_report_key(p_data->input_dev, M1120_EVENT_CODE, 1);
    input_sync(p_data->input_dev);
    input_report_key(p_data->input_dev, M1120_EVENT_CODE, 0);
    input_sync(p_data->input_dev);

    if( p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) 
    {
        printk("%s is_back:%d cur raw:%d \n", __func__, g_is_back, raw);

        m1120_update_interrupt_threshold(&p_data->client->dev);
#ifdef M1120_STATUS_SIGNAL
        if (async_queue)
            kill_fasync(&async_queue, SIGIO, POLL_IN); 
#endif
        enable_irq(p_m1120_data->irq);
    } 
    else 
    {
        schedule_delayed_work(&p_data->work, delay);
    }
}

/* functions for interrupt handler */
static irqreturn_t m1120_irq_handler(int irq, void *dev_id)
{
    printk(" %s  \n ", __FUNCTION__);
    
    disable_irq_nosync(p_m1120_data->irq);
    if(p_m1120_data != NULL)
    {
        schedule_delayed_work(&p_m1120_data->work, 0);
    }
    return IRQ_HANDLED;
}

/* functions for configuring or getting configured status */
static void m1120_get_reg(struct device *dev, int* regdata)
{
    struct i2c_client *client = to_i2c_client(dev);
    int err;

    u8 rega = (((*regdata) >> 8) & 0xFF);
    u8 regd = 0;
    err = m1120_i2c_get_reg(client, rega, &regd);

    *regdata = 0;
    *regdata |= (err==0) ? 0x0000 : 0xFF00;
    *regdata |= regd;
}

static void m1120_set_reg(struct device *dev, int* regdata)
{
    struct i2c_client *client = to_i2c_client(dev);
    int err;

    u8 rega = (((*regdata) >> 8) & 0xFF);
    u8 regd = *regdata&0xFF;
    err = m1120_i2c_set_reg(client, rega, regd);

    *regdata = 0;
    *regdata |= (err==0) ? 0x0000 : 0xFF00;
}


static int m1120_get_enable(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    return atomic_read(&p_data->atm.enable);
}

static void m1120_set_enable(struct device *dev, int enable)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    int delay = m1120_get_delay(dev);

    mutex_lock(&p_data->mtx.enable);

    if (enable) {                   /* enable if state will be changed */
        if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1)) {
            m1120_set_operation_mode(&p_m1120_data->client->dev, OPERATION_MODE_MEASUREMENT);
            schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
        }
    } else {                        /* disable if state will be changed */
        if (atomic_cmpxchg(&p_data->atm.enable, 1, 0)) {
            cancel_delayed_work_sync(&p_data->work);
            m1120_set_operation_mode(&p_m1120_data->client->dev, OPERATION_MODE_POWERDOWN);
        }
    }
    atomic_set(&p_data->atm.enable, enable);

    mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_delay(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    int delay = 0;

    delay = atomic_read(&p_data->atm.delay);

    return delay;
}

static void m1120_set_delay(struct device *dev, int delay)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    if(delay<M1120_DELAY_MIN) delay = M1120_DELAY_MIN;
    atomic_set(&p_data->atm.delay, delay);

    mutex_lock(&p_data->mtx.enable);

    if (m1120_get_enable(dev)) {
        cancel_delayed_work_sync(&p_data->work);
        schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
    }

    mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_debug(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    return atomic_read(&p_data->atm.debug);
}

static void m1120_set_debug(struct device *dev, int debug)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    atomic_set(&p_data->atm.debug, debug);
}

static int m1120_clear_interrupt(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    int ret = 0;

    ret = m1120_i2c_set_reg(p_data->client, M1120_REG_PERSINT, p_data->reg.map.persint | 0x01);

    return ret;
}

void m1120_convdata_short_to_2byte(u8 opf, short x, unsigned char *hbyte, unsigned char *lbyte)
{
    if( (opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
        /* 8 bit resolution */
        if(x<-128) x=-128;
        else if(x>127) x=127;

        if(x>=0) {
            *lbyte = x & 0x7F;
        } else {
            *lbyte = ( (0x80 - (x*(-1))) & 0x7F ) | 0x80;
        }
        *hbyte = 0x00;
    } else {
        /* 10 bit resolution */
        if(x<-512) x=-512;
        else if(x>511) x=511;

        if(x>=0) {
            *lbyte = x & 0xFF;
            *hbyte = (((x&0x100)>>8)&0x01) << 6;
        } else {
            *lbyte = (0x0200 - (x*(-1))) & 0xFF;
            *hbyte = ((((0x0200 - (x*(-1))) & 0x100)>>8)<<6) | 0x80;
        }
    }
}

short m1120_convdata_2byte_to_short(u8 opf, unsigned char hbyte, unsigned char lbyte)
{
    short x;

    if( (opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
        /* 8 bit resolution */
        x = lbyte & 0x7F;
        if(lbyte & 0x80) {
            x -= 0x80;
        }
    } else {
        /* 10 bit resolution */
        x = ( ( (hbyte & 0x40) >> 6) << 8 ) | lbyte;
        if(hbyte&0x80) {
            x -= 0x200;
        }
    }

    return x;
}

static int m1120_update_interrupt_threshold(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 lthh, lthl, hthh, hthl;
    //short raw;
    int err;
    printk("%s  low:%d   high:%d  \n", __FUNCTION__, p_data->thrlow, p_data->thrhigh);
    err = m1120_clear_interrupt(dev);
    //if(err) return err;

    if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) 
    {
        if(p_data->reg.map.intsrs & M1120_VAL_INTSRS_INTTYPE_BESIDE) 
        {
            if(g_is_back == 1) 
            {
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrlow, &lthh, &lthl);
            } 
            else
            {
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrhigh, &hthh, &hthl);
                m1120_convdata_short_to_2byte(p_data->reg.map.opf, -512, &lthh, &lthl);
            }               
        } 
        else 
        {
            // to do another condition
        }

        err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHH, hthh);
        if(err) return err;
        err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHL, hthl);
        if(err) return err;
        err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHH, lthh);
        if(err) return err;
        err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHL, lthl);
        if(err) return err;

        if(m1120_get_debug(dev)) {
            mxinfo(&client->dev, "threshold : (0x%02X%02X, 0x%02X%02X)\n", hthh, hthl, lthh, lthl);
        }
    }

    return err;
}

static int m1120_set_operation_mode(struct device *dev, int mode)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 opf = p_data->reg.map.opf;
    int ret = -1;

    switch(mode) {
        case OPERATION_MODE_POWERDOWN:
            opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
            ret = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
            mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_POWERDOWN");
            break;
        case OPERATION_MODE_MEASUREMENT:
            opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
            opf |= M1120_VAL_OPF_HSSON_ON;
            ret = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
            mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_MEASUREMENT");
            break;
        case OPERATION_MODE_FUSEROMACCESS:
            opf |= M1120_VAL_OPF_EFRD_ON;
            opf |= M1120_VAL_OPF_HSSON_ON;
            ret = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
            mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_FUSEROMACCESS");
            break;
    }

    return ret;
}

static int m1120_set_detection_mode(struct device *dev, u8 mode)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    u8 data;
    int err = 0;

    if(mode & M1120_DETECTION_MODE_INTERRUPT) 
    {
        /* config threshold */
        m1120_update_interrupt_threshold(dev);

        if(!p_data->irq_enabled) 
        {
            /* write intsrs */
            data = p_data->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;
            err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
            if(err) return err;
            /* enable irq */
            err = request_irq(p_data->irq, &m1120_irq_handler, IRQ_TYPE_LEVEL_LOW, M1120_DRIVER_NAME, 0);//M1120_IRQ_NAME
            
            printk("%s  request irq ok  err:%d \n", __FUNCTION__, err);
            
            m1120_clear_interrupt(dev);
            enable_irq_wake(p_data->irq);
            p_data->irq_enabled = 1;
        }
    } 
    else 
    {
        if(p_data->irq_enabled) 
        {
            /* write intsrs */
            data = p_data->reg.map.intsrs & (0xFF - M1120_DETECTION_MODE_INTERRUPT);
            err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
            if(err) return err;

            /* disable irq */
            disable_irq(p_data->irq);
            free_irq(p_data->irq, NULL);
            p_data->irq_enabled = 0;
        }
    }

    return 0;
}

static int m1120_init_device(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    int err = -1;

    /* (1) vdd and vid power up */
    err = m1120_set_power(dev, 1);
    if(err) {
        mxerr(&client->dev, "m1120 power-on was failed (%d)", err);
        return err;
    }

    /* (2) init variables */
    atomic_set(&p_data->atm.enable, 0);
    atomic_set(&p_data->atm.delay, M1120_DELAY_MIN);
    atomic_set(&p_data->atm.debug, 1);
    p_data->calibrated_data = 0;
    p_data->last_data = 0;
    p_data->irq_enabled = 0;
    p_data->thrhigh = M1120_DETECT_RANGE_HIGH;
    p_data->thrlow = M1120_DETECT_RANGE_LOW;
    m1120_set_delay(&client->dev, M1120_DELAY_MAX);
    m1120_set_debug(&client->dev, 0);

    /* (3) reset registers */
    err = m1120_reset_device(dev);
    if(err) {
        mxerr(&client->dev, "m1120_reset_device was failed (%d)", err);
        return err;
    }

    mxinfo(&client->dev, "initializing device was success");

    return 0;
}

static int m1120_reset_device(struct device *dev)
{
    int	err = 0;
    u8	id = 0xFF;

    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    if( (p_data == NULL) || (p_data->client == NULL) ) return -ENODEV;

    err = m1120_i2c_set_reg(p_data->client, M1120_REG_SRST, M1120_VAL_SRST_RESET);
    if(err) {
        mxerr(&client->dev, "sw-reset was failed(%d)", err);
        return err;
    }
    msleep(5); // wait 5ms
    dbg("wait 5ms after vdd power up");

    err = m1120_i2c_get_reg(p_data->client, M1120_REG_DID, &id);
    if (err < 0) return err;
    if (id != M1120_VAL_DID) {
        mxerr(&client->dev, "current device id(0x%02X) is not M1120 device id(0x%02X)", id, M1120_VAL_DID);
        return -ENXIO;
    }

    printk("%s dev id:%d \n", __FUNCTION__, id);

    g_is_back = 1;

    p_data->reg.map.persint = M1120_PERSISTENCE_COUNT;
    p_data->reg.map.intsrs = M1120_DETECTION_MODE | M1120_SENSITIVITY_TYPE;
    if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
        p_data->reg.map.intsrs |= M1120_INTERRUPT_TYPE;
    }
    p_data->reg.map.opf = M1120_OPERATION_FREQUENCY | M1120_OPERATION_RESOLUTION;

    err = m1120_set_operation_mode(dev, OPERATION_MODE_MEASUREMENT);//OPERATION_MODE_POWERDOWN  
    if(err) {
        mxerr(&client->dev, "m1120_set_detection_mode was failed(%d)", err);
        return err;
    }

    return err;
}

static int m1120_get_calibrated_data(struct device *dev, int* data)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    int err = 0;
    short adc = 0;

    if(p_data == NULL) 
        err = -ENODEV;

    msleep(M1120_DELAY_FOR_READY);
    err = m1120_measure(p_data, &adc);

    *data = p_data->calibrated_data = adc;

    return err;
}

static int m1120_measure(m1120_data_t *p_data, short *raw)
{
    struct i2c_client *client = p_data->client;
    int err = 0;
    u8 buf[3];
    static short adc = 0;

    // (1) read data
    err = m1120_i2c_read(client, M1120_REG_ST1, buf, sizeof(buf));
    if(err) return err;

    // (2) collect data
    if(buf[0] & 0x01) {
        adc = m1120_convdata_2byte_to_short(p_data->reg.map.opf, buf[2], buf[1]);
    } else {
        mxerr(&client->dev, "st1(0x%02X) is not DRDY", buf[0]);
        err = -1;
    }

    *raw = adc;

    if(m1120_get_debug(&client->dev)) {
        printk("raw data (%d)\n", *raw);
    }

    return err;
}

/* *************************************************
   input device interface
 ************************************************* */
static int m1120_input_dev_init(m1120_data_t *p_data)
{
    struct input_dev *dev;
    int err;

    dev = input_allocate_device();
    if (!dev) {
        return -ENOMEM;
    }
    dev->name = M1120_DRIVER_NAME;
    dev->id.bustype = BUS_I2C;

    input_set_drvdata(dev, p_data);
    input_set_capability(dev, M1120_EVENT_TYPE, M1120_EVENT_CODE);

    err = input_register_device(dev);
    if (err < 0) {
        input_free_device(dev);
        return err;
    }

    p_data->input_dev = dev;

    return 0;
}

static void m1120_input_dev_terminate(m1120_data_t *p_data)
{
    struct input_dev *dev = p_data->input_dev;

    input_unregister_device(dev);
    input_free_device(dev);
}


#ifdef M1120_STATUS_SIGNAL

static int pswitch_fasync(int fd, struct file * filp, int on) 
{
    return fasync_helper(fd, filp, on, &async_queue);
}

#endif


/* *************************************************
   misc device interface
 ************************************************* */

static int m1120_misc_dev_open( struct inode*, struct file* );
static int m1120_misc_dev_release( struct inode*, struct file* );
static long m1120_misc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg);
static ssize_t m1120_misc_dev_read( struct file *filp, char *buf, size_t count, loff_t *ofs );
static ssize_t m1120_misc_dev_write( struct file *filp, const char *buf, size_t count, loff_t *ofs );
static unsigned int m1120_misc_dev_poll( struct file *filp, struct poll_table_struct *pwait );

static struct file_operations m1120_misc_dev_fops =
{
    .owner = THIS_MODULE,
    .open = m1120_misc_dev_open,
    .unlocked_ioctl = m1120_misc_dev_ioctl,
    .release = m1120_misc_dev_release,
    .read = m1120_misc_dev_read,
    .write = m1120_misc_dev_write,
#ifdef M1120_STATUS_SIGNAL    
    .fasync = pswitch_fasync,    
#endif	
    .poll = m1120_misc_dev_poll,
};

static struct miscdevice m1120_misc_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = M1120_DRIVER_NAME,
    .fops = &m1120_misc_dev_fops,
};
/* m1120 misc device file operation */
static int m1120_misc_dev_open( struct inode* inode, struct file* file)
{
    return 0;
}

static int m1120_misc_dev_release( struct inode* inode, struct file* file)
{
#ifdef M1120_STATUS_SIGNAL
    fasync_helper(-1, file, 0, &async_queue);
#endif
    return 0;
}

static long m1120_misc_dev_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    void __user *argp = (void __user *)arg;
    int kbuf = 0;
    int caldata = 0;
    RotorDetectionCaliData  cali_data;

    switch( cmd ) {
        case RHALL_IOCTL_SET_ENABLE:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_SET_ENABLE(%d)\n", kbuf);
            m1120_set_enable(&p_m1120_data->client->dev, kbuf);
            break;
        case RHALL_IOCTL_GET_ENABLE:
            kbuf = m1120_get_enable(&p_m1120_data->client->dev);
            dbg("RHALL_IOCTL_GET_ENABLE(%d)\n", kbuf);
            if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
            break;
        case RHALL_IOCTL_SET_DELAY:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_SET_DELAY(%d)\n", kbuf);
            m1120_set_delay(&p_m1120_data->client->dev, kbuf);
            break;
        case RHALL_IOCTL_GET_DELAY:
            kbuf = m1120_get_delay(&p_m1120_data->client->dev);
            dbg("RHALL_IOCTL_GET_DELAY(%d)\n", kbuf);
            if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
            break;
        case RHALL_IOCTL_SET_CALIBRATION:               //use
            dbg("RHALL_IOCTL_SET_CALIBRATION\n");
            if(copy_from_user(&cali_data, argp, sizeof(cali_data))) return -EFAULT;
            if (cali_data.cali_flag != 1 || cali_data.lowthd > cali_data.highthd ) 
            {
                dbg("RHALL_IOCTL_SET_CALIBRATION para is invalid. \n");
                break;
            }
            p_m1120_data->thrlow = cali_data.lowthd;
            p_m1120_data->thrhigh= cali_data.highthd;
            g_is_back = 1;
            m1120_update_interrupt_threshold(&p_m1120_data->client->dev);
            break;
        case RHALL_IOCTL_GET_CALIBRATED_DATA:           //use
            printk("RHALL_IOCTL_GET_CALIBRATED_DATA");
            kbuf = m1120_get_calibrated_data(&p_m1120_data->client->dev, &caldata);
            if(copy_to_user(argp, &caldata, sizeof(caldata))) return -EFAULT;
            printk("calibrated data (%d)\n", caldata);
            break;
        case RHALL_IOCTL_GET_DATA:                      //use
            printk("RHALL_IOCTL_GET_DATA");
            kbuf = !g_is_back;
            if(copy_to_user(argp, &kbuf, sizeof(caldata))) return -EFAULT;
            printk("current hall state (%d)\n", kbuf);
            break;        
        case RHALL_IOCTL_SET_REG:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_SET_REG([0x%02X] %02X", (u8)((kbuf>>8)&0xFF), (u8)(kbuf&0xFF));
            m1120_set_reg(&p_m1120_data->client->dev, &kbuf);
            dbgn(" (%s))\n", (kbuf&0xFF00)?"Not Ok":"Ok");
            if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
            break;
        case RHALL_IOCTL_GET_REG:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_GET_REG([0x%02X]", (u8)((kbuf>>8)&0xFF) );
            m1120_get_reg(&p_m1120_data->client->dev, &kbuf);
            dbgn(" 0x%02X (%s))\n", (u8)(kbuf&0xFF), (kbuf&0xFF00)?"Not Ok":"Ok");
            if(copy_to_user(argp, &kbuf, sizeof(kbuf))) return -EFAULT;
            break;
        case RHALL_IOCTL_SET_INTERRUPT:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_SET_INTERRUPT(%d)\n", kbuf);
            if(kbuf) {
                m1120_set_detection_mode(&p_m1120_data->client->dev, M1120_DETECTION_MODE_INTERRUPT);
            } else {
                m1120_set_detection_mode(&p_m1120_data->client->dev, M1120_DETECTION_MODE_POLLING);
            }
            break;
        case RHALL_IOCTL_GET_INTERRUPT:
            kbuf = (p_m1120_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) ? 1 : 0 ;
            dbg("RHALL_IOCTL_GET_INTERRUPT(%d)\n", kbuf);
            if(copy_to_user(argp, &kbuf, sizeof(kbuf)));
            break;
        case RHALL_IOCTL_SET_THRESHOLD_HIGH:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_SET_THRESHOLD_HIGH(%d)\n", kbuf);
            p_m1120_data->thrhigh = kbuf;
            break;
        case RHALL_IOCTL_GET_THRESHOLD_HIGH:
            kbuf = p_m1120_data->thrhigh;
            dbg("RHALL_IOCTL_GET_THRESHOLD_HIGH(%d)\n", kbuf);
            if(copy_to_user(argp, &kbuf, sizeof(kbuf)));
            break;
        case RHALL_IOCTL_SET_THRESHOLD_LOW:
            if(copy_from_user(&kbuf, argp, sizeof(kbuf))) return -EFAULT;
            dbg("RHALL_IOCTL_SET_THRESHOLD_LOW(%d)\n", kbuf);
            p_m1120_data->thrlow = kbuf;
            break;
        case RHALL_IOCTL_GET_THRESHOLD_LOW:
            kbuf = p_m1120_data->thrlow;
            dbg("RHALL_IOCTL_GET_THRESHOLD_LOW(%d)\n", kbuf);
            if(copy_to_user(argp, &kbuf, sizeof(kbuf)));
            break;
        default:
            return -ENOTTY;
    }

    return ret;
}

static ssize_t m1120_misc_dev_read( struct file *filp, char *buf, size_t count, loff_t *ofs )
{
    return 0;
}

static ssize_t m1120_misc_dev_write( struct file *filp, const char *buf, size_t count, loff_t *ofs )
{
    return 0;
}

static unsigned int m1120_misc_dev_poll( struct file *filp, struct poll_table_struct *pwait )
{
    return 0;
}

/* *************************************************
   sysfs attributes
 ************************************************* */
static ssize_t m1120_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", m1120_get_enable(dev));
}

static ssize_t m1120_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);

    if ((enable == 0) || (enable == 1)) {
        m1120_set_enable(dev, enable);
    }

    return count;
}

static ssize_t m1120_delay_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", m1120_get_delay(dev));
}

static ssize_t m1120_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long delay = simple_strtoul(buf, NULL, 10);

    if (delay > M1120_DELAY_MAX) {
        delay = M1120_DELAY_MAX;
    }

    m1120_set_delay(dev, delay);

    return count;
}

static ssize_t m1120_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", m1120_get_debug(dev));
}

static ssize_t m1120_debug_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long debug = simple_strtoul(buf, NULL, 10);

    m1120_set_debug(dev, debug);

    return count;
}

static ssize_t m1120_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    return 0;
}

static ssize_t m1120_all_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i, err;
    u8 val;
    u8 buffer[512] = {0};
    u8 temp_buf[20] = {0};
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    for (i = 0; i <= 0x12; i++)
    {
        memset(temp_buf, 0, sizeof(temp_buf));
        err = m1120_i2c_get_reg(p_data->client, i, &val);
        if (err < 0)
        {
            return sprintf(buf, "read reg error!\n");
        }
        sprintf(temp_buf,  "reg 0x%x:0x%x\n", i, val);
        strcat(buffer, temp_buf);
    }

    return sprintf(buf, "%s\n", buffer);
}

static ssize_t m1120_adc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    short adc = 0;
    int err;
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);
    msleep(M1120_DELAY_FOR_READY);
    err = m1120_measure(p_data, &adc);
    return sprintf(buf, "%d\n", adc);
}

static ssize_t m1120_set_reg_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int reg, var;
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    if (sscanf(buf, "%x:%x", &reg, &var) == 2)
    {
        m1120_i2c_set_reg(p_data->client, reg, var);
    }

    return count;
}

static ssize_t m1120_thd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    m1120_data_t *p_data = i2c_get_clientdata(client);

    return sprintf(buf, "lowthd:%d highthd:%d \n", p_data->thrlow,  p_data->thrhigh);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
        m1120_enable_show, m1120_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
        m1120_delay_show, m1120_delay_store);
static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR|S_IWGRP,
        m1120_debug_show, m1120_debug_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
        NULL, m1120_wake_store);
static DEVICE_ATTR(allreg, S_IRUGO | S_IWUSR | S_IWGRP,
        m1120_all_reg_show, NULL);
static DEVICE_ATTR(adc, S_IRUGO | S_IWUSR | S_IWGRP,
        m1120_adc_show, NULL);
static DEVICE_ATTR(set_reg, S_IWUSR | S_IWGRP,
        NULL, m1120_set_reg_store);
static DEVICE_ATTR(thd, S_IRUGO | S_IWUSR | S_IWGRP,
        m1120_thd_show, NULL);

static struct attribute *m1120_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_debug.attr,
    &dev_attr_wake.attr,
    &dev_attr_allreg.attr,	
    &dev_attr_adc.attr,       
    &dev_attr_set_reg.attr,
    &dev_attr_thd.attr,      
    NULL
};

static struct attribute_group m1120_attribute_group = {
    .attrs = m1120_attributes
};

static ssize_t rotor_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
    char page[8]; 	
    char *p = page;	
    int len = 0; 	

    printk("%s, hall_state = %d\n", __func__, !g_is_back);
        
    p += sprintf(p, "%d\n", !g_is_back);	
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf,page,len < count ? len  : count))		
        return -EFAULT;	
    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;
}

#if 0
static ssize_t rotor_node_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
    char tmp[32] = {0};	
    int ret;		
    if (count > 2)		
        return -EINVAL;		
    ret = copy_from_user(tmp, buf, 32);

    sscanf(tmp, "%d", &g_is_back);	

    return count;	
}
#endif
static struct file_operations rotor_node_ctrl = {
    .read = rotor_node_read,
    //.write = rotor_node_write,  
};


#if 0
static void m1120_printk_allreg(void)
{
    u8 val = 0;
    int i = 0;
    for (i = 0; i <= 0x12; i++)
    {
        val = 0;
        m1120_i2c_get_reg(p_m1120_data->client, i, &val);
        printk("read reg:0x%x  val:0x%x \n", i, val);  
    }
}
#endif

void m1120_get_info_from_dts(struct device *dev, m1120_data_t* p_data)
{
    int rc;	
    struct device_node *np;
    np = dev->of_node;

    p_data->igpio = of_get_named_gpio(np, "dhall,irq-gpio", 0);
    p_data->irq= gpio_to_irq(p_data->igpio);

    printk("GPIO %d use for DHALL interrupt  irq:%d \n",p_data->igpio, p_data->irq);

    p_data->power_vdd = regulator_get(&p_data->client->dev, "vdd_2v8");
    if (IS_ERR(p_data->power_vdd)) 
    {
        rc = PTR_ERR(p_data->power_vdd);
        dev_err(&p_data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
    }	

    p_data->power_vi2c = regulator_get(&p_data->client->dev, "vcc_i2c_1v8");
    if (IS_ERR(p_data->power_vi2c)) 
    {
        rc = PTR_ERR(p_data->power_vi2c);
        dev_err(&p_data->client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
    }      
}

int m1120_i2c_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    m1120_platform_data_t	*p_platform;
    m1120_data_t			*p_data = NULL;
    int						err = 0;

    dbg_func_in();
    printk("m1120_i2c_drv_probe start...\n");
    //goto test_ok;

    p_data = kzalloc(sizeof(m1120_data_t), GFP_KERNEL);
    if (!p_data) {
        mxerr(&client->dev, "kernel memory alocation was failed");
        err = -ENOMEM;
        goto error_0;
    }

    mutex_init(&p_data->mtx.enable);
    mutex_init(&p_data->mtx.data);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        mxerr(&client->dev, "i2c_check_functionality was failed");
        err = -ENODEV;
        goto error_1;
    }
    i2c_set_clientdata(client, p_data);
    p_data->client = client;

    p_platform = client->dev.platform_data;

    INIT_DELAYED_WORK(&p_data->work, m1120_work_func);

    m1120_get_info_from_dts(&client->dev, p_data);
    if(p_data->igpio != -1) 
    {
        err = gpio_request(p_data->igpio, M1120_IRQ_NAME);
        if (err){
            mxerr(&client->dev, "gpio_request was failed(%d)", err); 
            goto error_1;
        }

        err = gpio_direction_input(p_data->igpio);
        if (err < 0) {
            mxerr(&client->dev, "gpio_direction_input was failed(%d)", err);
            goto error_2;
        }       
    }

    printk("%s  irq:%d  irq-gpio:%d \n", __FUNCTION__, p_data->irq, gpio_get_value(p_data->igpio));

    err = m1120_init_device(&p_data->client->dev);
    if(err) {
        mxerr(&client->dev, "m1120_init_device was failed(%d)", err);
        goto error_1;
    }
    mxinfo(&client->dev, "%s was found", id->name);

    err = m1120_input_dev_init(p_data);
    if(err) {
        mxerr(&client->dev, "m1120_input_dev_init was failed(%d)", err);
        goto error_1;
    }
    mxinfo(&client->dev, "%s was initialized", M1120_DRIVER_NAME);

    err = sysfs_create_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);
    if(err) {
        mxerr(&client->dev, "sysfs_create_group was failed(%d)", err);
        goto error_3;
    }

    rotordir = proc_create(rotor_node_name, 0664, NULL, &rotor_node_ctrl); 
    if (rotordir == NULL)
    {
        printk(" create proc/%s fail\n", rotor_node_name);
        goto error_3;
    }

    err = misc_register(&m1120_misc_dev);
    if(err) {
        mxerr(&client->dev, "misc_register was failed(%d)", err);
        goto error_4;
    }

    p_m1120_data = p_data;

    err = m1120_set_detection_mode(&client->dev, M1120_DETECTION_MODE);
    if(err) {
        mxerr(&client->dev, "m1120_set_detection_mode was failed(%d)", err);
        return err;
    }

    printk("%s : %s was probed.\n", __func__, M1120_DRIVER_NAME);

    return 0;

error_4:
    sysfs_remove_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);

error_3:
    m1120_input_dev_terminate(p_data);

error_2:
    if(p_data->igpio != -1) {
        gpio_free(p_data->igpio);
    }

error_1:
    kfree(p_data);

error_0:

    return err;
}

static int m1120_i2c_drv_remove(struct i2c_client *client)
{
    m1120_data_t *p_data = i2c_get_clientdata(client);

    m1120_set_enable(&client->dev, 0);
    misc_deregister(&m1120_misc_dev);
    sysfs_remove_group(&p_data->input_dev->dev.kobj, &m1120_attribute_group);
    m1120_input_dev_terminate(p_data);
    if(p_data->igpio!= -1) {
        gpio_free(p_data->igpio);
    }
    kfree(p_data);

    return 0;
}

static int m1120_i2c_drv_suspend(struct device *dev)
{
#if 0 /* delete it by lauson. */
    m1120_data_t *p_data = dev_get_drvdata(dev);

    dbg_func_in();

    mutex_lock(&p_data->mtx.enable);

    if (m1120_get_enable(dev)) {
        if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
            m1120_set_operation_mode(dev, OPERATION_MODE_MEASUREMENT);
        } else {
            cancel_delayed_work_sync(&p_data->work);
            m1120_set_detection_mode(dev, M1120_DETECTION_MODE_INTERRUPT);
        }
    }

    mutex_unlock(&p_data->mtx.enable);
#endif 

    dbg_func_out();

    return 0;
}

static int m1120_i2c_drv_resume(struct device *dev)
{
#if 0 /* delete it by lauson. */
    m1120_data_t *p_data = dev_get_drvdata(dev);

    dbg_func_in();

    mutex_lock(&p_data->mtx.enable);

    if (m1120_get_enable(dev)) {
        if(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
            m1120_set_detection_mode(dev, M1120_DETECTION_MODE_POLLING);
            schedule_delayed_work(&p_data->work, msecs_to_jiffies(m1120_get_delay(dev)));
        }
    }

    mutex_unlock(&p_data->mtx.enable);
#endif 

    dbg_func_out();

    return 0;
}

static const struct i2c_device_id m1120_i2c_drv_id_table[] = {
    {M1120_DRIVER_NAME, 0 },
    { }
};

static const struct dev_pm_ops m1120_pm_ops = {
    .suspend = m1120_i2c_drv_suspend,
    .resume = m1120_i2c_drv_resume,
};	

static struct i2c_driver m1120_driver = {
    .driver = {
        .name	= M1120_DRIVER_NAME,
        .of_match_table =  dhall_device_id,
        .pm = &m1120_pm_ops,
    },
    .probe		= m1120_i2c_drv_probe,
    .remove		= m1120_i2c_drv_remove,
    .id_table	= m1120_i2c_drv_id_table,
    //.suspend	= m1120_i2c_drv_suspend,
    //.resume		= m1120_i2c_drv_resume,
};

static int __init m1120_driver_init(void)
{
    printk(KERN_INFO "%s .... \n", __func__);

    if(i2c_add_driver(&m1120_driver)!=0) 
    {
        printk("unable to add m1120 i2c driver.\n");
        return -1;
    }	
    return 0;    
}
module_init(m1120_driver_init);

static void __exit m1120_driver_exit(void)
{
    printk(KERN_INFO "%s\n", __func__);
    i2c_del_driver(&m1120_driver);
}
module_exit(m1120_driver_exit);

MODULE_AUTHOR("shpark <seunghwan.park@magnachip.com>");
MODULE_VERSION(M1120_DRIVER_VERSION);
MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");

