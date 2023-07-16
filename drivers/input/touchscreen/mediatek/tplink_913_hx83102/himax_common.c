/* Himax Android Driver Sample Code for common functions
*
* Copyright (C) 2017 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include "himax_common.h"
#include "himax_ic.h"

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)
#define FRAME_COUNT 5

#ifdef HX_RST_PIN_FUNC
extern void himax_ic_reset(uint8_t loadconfig,uint8_t int_off);
#endif

#ifdef HX_THRESHOLD_ADJUST_ENABLE
extern void himax_set_sensitivity_enable(struct i2c_client *client, uint8_t sensitivity);
#endif

extern void himax_rst_gpio_set(int pinnum, uint8_t value);

#if defined(HX_AUTO_UPDATE_FW)
unsigned char i_CTPM_FW[]=
{
#include "HX83100_Firmware_Version_FF23.i"
};
#endif

#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)||defined(HX_THRESHOLD_ADJUST_ENABLE)
extern void himax_resend_cmd_func(bool suspended);
extern void himax_rst_cmd_recovery_func(bool suspended);
#endif

struct himax_ts_data *private_ts;
struct himax_ic_data *ic_data;
struct himax_report_data *hx_touch_data;

static int		HX_TOUCH_INFO_POINT_CNT = 0;

unsigned long	FW_VER_MAJ_FLASH_ADDR;
unsigned long	FW_VER_MIN_FLASH_ADDR;
unsigned long	CFG_VER_MAJ_FLASH_ADDR;
unsigned long	CFG_VER_MIN_FLASH_ADDR;
unsigned long	CID_VER_MAJ_FLASH_ADDR;
unsigned long	CID_VER_MIN_FLASH_ADDR;
//unsigned long	PANEL_VERSION_ADDR;

unsigned long	FW_VER_MAJ_FLASH_LENG;
unsigned long	FW_VER_MIN_FLASH_LENG;
unsigned long	CFG_VER_MAJ_FLASH_LENG;
unsigned long	CFG_VER_MIN_FLASH_LENG;
unsigned long	CID_VER_MAJ_FLASH_LENG;
unsigned long	CID_VER_MIN_FLASH_LENG;
//unsigned long	PANEL_VERSION_LENG;

unsigned long	FW_CFG_VER_FLASH_ADDR;

/* [lichenggang start]add proc dir for touchpanel*/
atomic_t double_enable;//[lichenggang start] add for gesture
atomic_t close_incall;//[lichenggang start] modify for psensor contorl tp when call
static int boot_mode = UNKNOWN_BOOT;//[lichenggang] add for boot mode judge
/* [lichenggang ]add for glove mode */

/* [lichenggang start]optimize gesture trajectory data*/
#define PIXEL_DISTANCE_WIDTH   96
#define PIXEL_DISTANCE_HEIGHT  96
#define GESTURE_LIMIT_WIDTH    17000
#define GESTURE_LIMIT_HEIGHT   17000
/* [lichenggang end] */


#ifdef HX_THRESHOLD_ADJUST_ENABLE
    uint8_t sensitivity_addr[4];
    uint8_t sensitivity_data[4];
    uint8_t sensitivity_addr1[4];
    uint8_t sensitivity_data1[4];
#endif
/* [lichenggang end] */
/* [lichenggang end] */

/* [lichenggang start] add for gesture coordinate */
#define GESTURE_O                           0x30
#define GESTURE_CW_O                        0x57
#define GESTURE_W                           0x31
#define GESTURE_M                           0x32
#define GESTURE_V                           0x54
#define GESTURE_LEFT_V                      0x51
#define GESTURE_RIGHT_V                     0x52
#define GESTURE_DOUBLECLICK                 0x24
#define GESTURE_DOUBLELINE                  0x28
#define GESTURE_CW_DOUBLELINE               0x27

#define UnkownGesture       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define RightVee            4   // >
#define LeftVee             5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Mgesture            12  // M
#define Wgesture            13  // W
//#define DownVee             3   // ^
//#define Left2RightSwip      8   // -->
//#define Right2LeftSwip      9   // <--
//#define Up2DownSwip         10  // |v
//#define Down2UpSwip         11  // |^



#ifdef HX_SMART_WAKEUP
unsigned short coordinate_report[13] = {0};
static u8 gesture_upload = 0;
#endif
/* [lichenggang end] */

#ifdef HX_AUTO_UPDATE_FW
int g_i_FW_VER = 0;
int g_i_CFG_VER = 0;
int g_i_CID_MAJ = 0; //GUEST ID
int g_i_CID_MIN = 0; //VER for GUEST
#endif

unsigned char	IC_TYPE = 11;
unsigned char	IC_CHECKSUM = 0;

#ifdef HX_ESD_RECOVERY
u8 HX_ESD_RESET_ACTIVATE = 0;
int hx_EB_event_flag = 0;
int hx_EC_event_flag = 0;
int hx_ED_event_flag = 0;
extern int g_zero_event_count;
#endif
u8 HX_HW_RESET_ACTIVATE = 0;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
extern int himax_touch_proc_init(void);
extern void himax_touch_proc_deinit(void);
extern void himax_touch_tp_info_init(void);// [lichenggang]add for tp_info node

//PROC-START
#ifdef  HX_TP_PROC_FLASH_DUMP
extern void himax_ts_flash_func(void);
extern void setFlashBuffer(void);
extern bool getFlashDumpGoing(void);
extern uint8_t getSysOperation(void);
extern void setSysOperation(uint8_t operation);
#endif

#ifdef HX_TP_PROC_DIAG
extern void himax_ts_diag_func(void);

extern int16_t *getMutualBuffer(void);
extern int16_t *getMutualNewBuffer(void);
extern int16_t *getMutualOldBuffer(void);
extern int16_t *getSelfBuffer(void);
extern uint8_t getXChannel(void);
extern uint8_t getYChannel(void);
extern uint8_t getDiagCommand(void);
extern void setXChannel(uint8_t x);
extern void setYChannel(uint8_t y);
extern void setMutualBuffer(void);
extern void setMutualNewBuffer(void);
extern void setMutualOldBuffer(void);
extern uint8_t diag_coor[128];
extern int himax_set_diag_cmd(struct himax_ic_data *ic_data,struct himax_report_data *hx_touch_data);
#ifdef HX_TP_PROC_2T2R
extern bool Is_2T2R;
extern int16_t *getMutualBuffer_2(void);
extern uint8_t getXChannel_2(void);
extern uint8_t getYChannel_2(void);
extern void setXChannel_2(uint8_t x);
extern void setYChannel_2(uint8_t y);
extern void setMutualBuffer_2(void);
/* [yanlin start] TP threshold test */
extern void setMutualMaxBuffer(void);//morgen
extern void setSelfMaxBuffer(void);
/* [yanlin end] */
#endif
#endif
//PROC-END
#endif

extern int himax_parse_dt(struct himax_ts_data *ts,
                          struct himax_i2c_platform_data *pdata);
extern bool himax_calculateChecksum(struct i2c_client *client, bool change_iref);
static uint8_t	AA_press = 0x00;
static uint8_t	EN_NoiseFilter = 0x00;
static uint8_t	Last_EN_NoiseFilter = 0x00;
static int	hx_point_num = 0;		// for himax_ts_work_func use
static int	p_point_num = 0xFFFF;
static int	tpd_key = 0x00;
static int	tpd_key_old = 0x00;
static int	probe_fail_flag = 0;
#ifdef HX_USB_DETECT_GLOBAL
bool USB_detect_flag = 0;
#endif

#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
                         unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#if defined(HX_PALM_REPORT)
int himax_palm_detect(uint8_t *buf);
#endif

#ifdef HX_GESTURE_TRACK
static int gest_pt_cnt;
static int gest_pt_x[GEST_PT_MAX_NUM];
static int gest_pt_y[GEST_PT_MAX_NUM];
static int gest_start_x=0,gest_start_y=0,gest_end_x=0,gest_end_y=0;
static int gest_width=0,gest_height=0,gest_mid_x=0,gest_mid_y=0;
static int gn_gesture_coor[16];
#endif


#if defined(HX_TP_PROC_SELF_TEST)|| defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
extern int g_self_test_entered;
#endif

int himax_report_data_init(void);
extern int himax_get_touch_data_size(void);
//void himax_HW_reset(uint8_t loadconfig,uint8_t int_off);
extern void himax_log_touch_data(uint8_t *buf,struct himax_report_data *hx_touch_data);
extern void himax_log_touch_event(int x,int y,int w,int loop_i,uint8_t EN_NoiseFilter,int touched);
extern void himax_log_touch_event_detail(struct himax_ts_data *ts,int x,int y,int w,int loop_i,uint8_t EN_NoiseFilter,int touched,uint16_t old_finger);

#if defined(HX_ESD_RECOVERY)
extern void himax_esd_ic_reset(void);
extern int himax_ic_esd_recovery(int hx_esd_event,int hx_zero_event,int length);
#endif

extern int himax_dev_set(struct himax_ts_data *ts);
extern int himax_input_register_device(struct input_dev *input_dev);


#ifdef HX_ZERO_FLASH
extern void himax_0f_operation(struct work_struct *work);
#endif

int himax_input_register(struct himax_ts_data *ts)
{
    int ret = 0;

    ret = himax_dev_set(ts);
    if(ret < 0)
        goto input_device_fail;

    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(KEY_F4, ts->input_dev->keybit); //[lichenggang]add for gesture key event

#if defined(HX_SMART_WAKEUP) || defined(HX_PALM_REPORT)
    set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
    set_bit(BTN_TOUCH, ts->input_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef	HX_PROTOCOL_A
    //ts->input_dev->mtsize = ts->nFinger_support;
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
                         0, 3, 0, 0);
#else
    set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
#if defined(HX_PROTOCOL_B_3PA)
    input_mt_init_slots(ts->input_dev, ts->nFinger_support,0);
#else
    input_mt_init_slots(ts->input_dev, ts->nFinger_support);
#endif
#endif

    I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
      ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
#ifndef	HX_PROTOCOL_A
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);
#endif

    if(himax_input_register_device(ts->input_dev) == 0)
        return NO_ERR;
    else
        ret = INPUT_REGISTER_FAIL;

input_device_fail:
    I("%s, input device register fail!\n",__func__);
    return ret;
}

static void calcDataSize(uint8_t finger_num)
{
    struct himax_ts_data *ts_data = private_ts;
    ts_data->coord_data_size = 4 * finger_num;
    ts_data->area_data_size = ((finger_num / 4) + (finger_num % 4 ? 1 : 0)) * 4;
    ts_data->coordInfoSize = ts_data->coord_data_size + ts_data->area_data_size + 4;
    ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;
    if(ts_data->raw_data_frame_size == 0)
    {
        E("%s: could NOT calculate! \n", __func__);
        return;
    }
    ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
                                  ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
                                 (((uint32_t)ts_data->x_channel * ts_data->y_channel +
                                   ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size)? 1 : 0;
    I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

void calculate_point_number(void)
{
    HX_TOUCH_INFO_POINT_CNT = ic_data->HX_MAX_PT * 4 ;

    if ( (ic_data->HX_MAX_PT % 4) == 0)
        HX_TOUCH_INFO_POINT_CNT += (ic_data->HX_MAX_PT / 4) * 4 ;
    else
        HX_TOUCH_INFO_POINT_CNT += ((ic_data->HX_MAX_PT / 4) +1) * 4 ;
}

#ifdef HX_AUTO_UPDATE_FW
static int i_update_FW(void)
{
    int upgrade_times = 0;
    unsigned char* ImageBuffer = i_CTPM_FW;
    int fullFileLength = sizeof(i_CTPM_FW);
    uint8_t ret = 0, result = 0;

    I("%s: i_fullFileLength = %d\n", __func__,fullFileLength);

    himax_int_enable(private_ts->client->irq,0);
update_retry:
    if(fullFileLength == FW_SIZE_32k)
    {
        ret = fts_ctpm_fw_upgrade_with_sys_fs_32k(private_ts->client,ImageBuffer,fullFileLength,false);
    }
    else if(fullFileLength == FW_SIZE_60k)
    {
        ret = fts_ctpm_fw_upgrade_with_sys_fs_60k(private_ts->client,ImageBuffer,fullFileLength,false);
    }
    else if (fullFileLength == FW_SIZE_64k)
    {
        ret = fts_ctpm_fw_upgrade_with_sys_fs_64k(private_ts->client,ImageBuffer,fullFileLength,false);
    }
    else if (fullFileLength == FW_SIZE_124k)
    {
        ret = fts_ctpm_fw_upgrade_with_sys_fs_124k(private_ts->client,ImageBuffer,fullFileLength,false);
    }
    else if (fullFileLength == FW_SIZE_128k)
    {
        ret = fts_ctpm_fw_upgrade_with_sys_fs_128k(private_ts->client,ImageBuffer,fullFileLength,false);
    }
    if(ret == 0)
    {
        upgrade_times++;
        E("%s: TP upgrade error, upgrade_times = %d\n", __func__, upgrade_times);
        if(upgrade_times < 3)
            goto update_retry;
        else
            result = -1;//upgrade fail
    }
    else
    {
        I("%s: tp upgrade ok and will read firmware version\n", __func__);
        himax_read_FW_ver(private_ts->client);
        himax_touch_information(private_ts->client);
        result = 1;//upgrade success
    }
#ifdef HX_RST_PIN_FUNC
    himax_ic_reset(true,false);
#else
    himax_sense_on(private_ts->client,0);

#endif
    himax_int_enable(private_ts->client->irq,1);
    return result;
}
#endif

int himax_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata)
{

    if (!client)
    {
        E("%s: Necessary parameters client are null!\n", __func__);
        return -1;
    }
    I("%s: initialization complete\n", __func__);
    return NO_ERR;
}

#ifdef HX_ESD_RECOVERY
void himax_esd_hw_reset(void)
{
    I("START_Himax TP: ESD - Reset\n");
#if defined(HX_TP_PROC_SELF_TEST) || defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
    if (g_self_test_entered == 1)
    {
        I("In self test ,not  TP: ESD - Reset\n");
        return;
    }
#endif
    HX_ESD_RESET_ACTIVATE = 1;
    himax_esd_ic_reset();

    I("END_Himax TP: ESD - Reset\n");
}
#endif

#ifdef HX_SMART_WAKEUP
#ifdef HX_GESTURE_TRACK
static void gest_pt_log_coordinate(int rx, int tx)
{
    //driver report x y with range 0 - 255 , we scale it up to x/y pixel
    gest_pt_x[gest_pt_cnt] = rx*(ic_data->HX_X_RES)/255;
    gest_pt_y[gest_pt_cnt] = tx*(ic_data->HX_Y_RES)/255;
}
#endif
static int himax_parse_wake_event(struct himax_ts_data *ts)
{
    uint8_t *buf;
#ifdef HX_GESTURE_TRACK
    int tmp_max_x=0x00,tmp_min_x=0xFFFF,tmp_max_y=0x00,tmp_min_y=0xFFFF;
    int gest_len;
#endif
    int i=0, check_FC = 0, gesture_flag = 0;

    buf = kzalloc(hx_touch_data->event_size*sizeof(uint8_t),GFP_KERNEL);
    memcpy(buf,hx_touch_data->hx_event_buf,hx_touch_data->event_size);

    pr_err("%s:buf[0~3] is 0x%x 0x%x 0x%x 0x%x\n", __func__, buf[0],buf[1],buf[2],buf[3]);

    for(i=0; i<GEST_PTLG_ID_LEN; i++)
    {
        if (check_FC==0)
        {
            if((buf[0] != 0x00) && ((buf[0] == 0x30)||(buf[0] == 0x57)||(buf[0] == 0x31)
                ||(buf[0] == 0x32)||(buf[0] == 0x54)||(buf[0] == 0x51)||(buf[0] == 0x52)
                ||(buf[0] == 0x24)||(buf[0] == 0x28)||(buf[0] == 0x27)))
            {
                check_FC = 1;
                gesture_flag = buf[i];
            }
            else
            {
                check_FC = 0;
                I("ID START at 0x%x , value = %x skip the event\n", i, buf[i]);
                break;
            }
        }
        else
        {
            if(buf[i]!=gesture_flag)
            {
                check_FC = 0;
                I("ID NOT the same %x != %x So STOP parse event\n", buf[i], gesture_flag);
                break;
            }
        }

        I("0x%2.2X ", buf[i]);
        if (i % 8 == 7)
            I("\n");
    }
    I("Himax gesture_flag= %x\n",gesture_flag );
    I("Himax check_FC is %d\n", check_FC);

    if (check_FC == 0)
        return 0;
    if(buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
            buf[GEST_PTLG_ID_LEN+1] != GEST_PTLG_HDR_ID2)
        return 0;

#ifdef HX_GESTURE_TRACK
    if(buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
            buf[GEST_PTLG_ID_LEN+1] == GEST_PTLG_HDR_ID2)
    {
        gest_len = buf[GEST_PTLG_ID_LEN+2];

        I("gest_len = %d ",gest_len);

        i = 0;
        gest_pt_cnt = 0;
        I("get coornidate start \n %s",__func__);
        while(i<(gest_len+1)/2)
        {
            gest_pt_log_coordinate(buf[GEST_PTLG_ID_LEN+4+i*2],buf[GEST_PTLG_ID_LEN+4+i*2+1]);
            i++;
            I("%s:buf[%d]=%d,buf[%d]=%d\n",__func__,GEST_PTLG_ID_LEN+4+i*2,buf[GEST_PTLG_ID_LEN+4+i*2],GEST_PTLG_ID_LEN+4+i*2+1,buf[GEST_PTLG_ID_LEN+4+i*2+1]); 
            I("gest_pt_x[%d] gest_pt_y[%d] = [%d %d]\n",gest_pt_cnt,gest_pt_cnt,gest_pt_x[gest_pt_cnt],gest_pt_y[gest_pt_cnt]);
            gest_pt_cnt +=1;
        }
        if(gest_pt_cnt)
        {
            for(i=0; i<gest_pt_cnt; i++)
            {
                /* [lichenggang start] filter out the error data form firmware*/
                if(gest_pt_x[i] == 0 || gest_pt_x[i] == 720 || gest_pt_y[i] == 0 || gest_pt_y[i] == 1440)
                    continue;
                /* [lichenggang end] */
                if(tmp_max_x<gest_pt_x[i])
                    tmp_max_x=gest_pt_x[i];
                if(tmp_min_x>gest_pt_x[i])
                    tmp_min_x=gest_pt_x[i];
                if(tmp_max_y<gest_pt_y[i])
                    tmp_max_y=gest_pt_y[i];
                if(tmp_min_y>gest_pt_y[i])
                    tmp_min_y=gest_pt_y[i];
            }
            I("gest_point x_min= %d, x_max= %d, y_min= %d, y_max= %d\n",tmp_min_x,tmp_max_x,tmp_min_y,tmp_max_y);

            gest_start_x=gest_pt_x[0];
            gest_start_y=gest_pt_y[0];
            gest_end_x=gest_pt_x[1];
            gest_end_y=gest_pt_y[1];
            gest_width = tmp_max_x - tmp_min_x;
            gest_height = tmp_max_y - tmp_min_y;
            gest_mid_x = (tmp_max_x + tmp_min_x)/2;
            gest_mid_y = (tmp_max_y + tmp_min_y)/2;

            /* [lichenggang start]optimize gesture trajectory data*/
            if (gesture_flag >= 0x30) {
                if (((gest_width * PIXEL_DISTANCE_WIDTH) < GESTURE_LIMIT_WIDTH)
                        || ((gest_height * PIXEL_DISTANCE_HEIGHT) < GESTURE_LIMIT_HEIGHT)) {
                    printk("%s: gesture data error\n", __func__);
                    return 0;
                }
            }
            /* [lichenggang end] */

            gn_gesture_coor[0] = gest_pt_x[0];//start
            gn_gesture_coor[1] = gest_pt_y[0];
            gn_gesture_coor[2] = gest_pt_x[1];//end
            gn_gesture_coor[3] = gest_pt_y[1];
            gn_gesture_coor[4] = gest_pt_x[2];//inflexion 1
            gn_gesture_coor[5] = gest_pt_y[2];
            gn_gesture_coor[6] = gest_pt_x[3];//inflexion 2
            gn_gesture_coor[7] = gest_pt_y[3];
            gn_gesture_coor[8] = gest_pt_x[4];//inflexion 3
            gn_gesture_coor[9] = gest_pt_y[4];
            gn_gesture_coor[10] = gest_pt_x[5];//inflexion 4
            gn_gesture_coor[11] = gest_pt_y[5];
            gn_gesture_coor[12] = gest_pt_x[6];
            gn_gesture_coor[13] = gest_pt_y[6];
            gn_gesture_coor[14] = gest_pt_x[7];
            gn_gesture_coor[15] = gest_pt_y[7];
            for (i = 0; i < 16; i++) {
                I("gn_gesture_coor[%d] = %d\n", i, gn_gesture_coor[i]);
            }
            if(( gesture_flag == 0x51 ) || ( gesture_flag == 0x52 ) || ( gesture_flag == 0x54 )){
                /*0x51 is "<", 0x52 is ">", 0x54 is "V" */
                coordinate_report[0] = gn_gesture_coor[0];//start_x
                coordinate_report[1] = gn_gesture_coor[1];//start_y
                coordinate_report[2] = gn_gesture_coor[2];//end_x
                coordinate_report[3] = gn_gesture_coor[3];//end_y
                coordinate_report[4] = gn_gesture_coor[4];//inflexion_x
                coordinate_report[5] = gn_gesture_coor[5];//inflexion_y
                coordinate_report[6] = 0;
                coordinate_report[7] = 0;
                coordinate_report[8] = 0;
                coordinate_report[9] = 0;
                coordinate_report[10] = 0;
                coordinate_report[11] = 0;
                coordinate_report[12] = 0;//indicate direction,not use in 913 project
            }else if(( gesture_flag == 0x31 ) || ( gesture_flag == 0x32 )){
                /*0x31 is "W", 0x32 is "M" */
                coordinate_report[0] = gn_gesture_coor[0];//start_x
                coordinate_report[1] = gn_gesture_coor[1];//start_y
                coordinate_report[2] = gn_gesture_coor[2];//end_x
                coordinate_report[3] = gn_gesture_coor[3];//end_y
                coordinate_report[4] = gn_gesture_coor[4];//inflexion_1_x
                coordinate_report[5] = gn_gesture_coor[5];//inflexion_1_y
                coordinate_report[6] = gn_gesture_coor[6];//inflexion_2_x
                coordinate_report[7] = gn_gesture_coor[7];//inflexion_2_y
                coordinate_report[8] = gn_gesture_coor[8];//inflexion_3_x
                coordinate_report[9] = gn_gesture_coor[9];//inflexion_3_y
                coordinate_report[10] = 0;
                coordinate_report[11] = 0;
                coordinate_report[12] = 0;//indicate direction,not use in 913 project
                /*process data for display */
                if(gn_gesture_coor[1] < gn_gesture_coor[3])
                    coordinate_report[1] = gn_gesture_coor[3];
                else
                    coordinate_report[3] = gn_gesture_coor[1];

                if(gn_gesture_coor[5] < gn_gesture_coor[9])
                    coordinate_report[5] = gn_gesture_coor[9];
                else
                    coordinate_report[9] = gn_gesture_coor[5];

                coordinate_report[6] = (coordinate_report[0] + coordinate_report[2])/2;
                coordinate_report[7] = (coordinate_report[1] + coordinate_report[3])/2;
                coordinate_report[4] = (coordinate_report[0] + coordinate_report[6])/2;
                coordinate_report[8] = (coordinate_report[2] + coordinate_report[6])/2;

            }else if(( gesture_flag == 0x28 ) || ( gesture_flag == 0x27 )){
            /* "||"gesture, gn_gesture_coor[4] is start_x and gn_gesture_coor[5] is start_y of second line,
                not use in coordinate_report*/
                coordinate_report[0] = gn_gesture_coor[0];//first line start_x
                coordinate_report[1] = gn_gesture_coor[1];//first line start_y
                coordinate_report[2] = gn_gesture_coor[2];//first line end_x
                coordinate_report[3] = gn_gesture_coor[3];//first line end_y
                coordinate_report[4] = gn_gesture_coor[0];//first line start_x
                coordinate_report[5] = gn_gesture_coor[1];//first line start_y
                coordinate_report[6] = gn_gesture_coor[6];//second line end_x
                coordinate_report[7] = gn_gesture_coor[7];//second line end_x
                coordinate_report[8] = gn_gesture_coor[8];
                coordinate_report[9] = gn_gesture_coor[9];
                coordinate_report[10] = 0;
                coordinate_report[11] = 0;
                coordinate_report[12] = 0; //indicate direction,not use in 913 project
            }else if(( gesture_flag == 0x30 ) || ( gesture_flag == 0x57 )){
                /*0x30 and 0x57 is "O",*/
                coordinate_report[0] = gn_gesture_coor[0];//start_x
                coordinate_report[1] = gn_gesture_coor[1];//start_y
                coordinate_report[2] = gn_gesture_coor[10];//right_x
                coordinate_report[3] = gn_gesture_coor[11];//right_y
                coordinate_report[4] = gn_gesture_coor[4];//top_x
                coordinate_report[5] = gn_gesture_coor[5];//top_y
                coordinate_report[6] = gn_gesture_coor[6];//left_x
                coordinate_report[7] = gn_gesture_coor[7];//left_y
                coordinate_report[8] = gn_gesture_coor[8];//bottom_x
                coordinate_report[9] = gn_gesture_coor[9];//bottom_y
                coordinate_report[10] = gn_gesture_coor[2];//end_x,not use
                coordinate_report[11] = gn_gesture_coor[3];//end_y,not use
                if(gesture_flag == 0x57)
                    coordinate_report[12] = 1;//indicate direction,anti-clockwise
                else
                    coordinate_report[12] = 0;//indicate direction,clockwise
            }
        }

    }
#endif

    return gesture_flag;
}

void himax_wake_check_func(void)
{
    int ret_event = 0;


    E("%s:enter\n",__func__);
    ret_event = himax_parse_wake_event(private_ts);
    E("gesture_id:ret_event=0x%x\n",ret_event);
    switch (ret_event)
    {
    case GESTURE_DOUBLECLICK:
        gesture_upload = DouTap;
        break;
    case GESTURE_V:
        gesture_upload = UpVee;
        break;
    case GESTURE_LEFT_V:
        gesture_upload = LeftVee;
        break;
    case GESTURE_RIGHT_V:
        gesture_upload = RightVee;
        break;
    case GESTURE_O:
        gesture_upload = Circle;
        break;
    case GESTURE_CW_O:
        gesture_upload = Circle;
        break;
    case GESTURE_DOUBLELINE:
        gesture_upload = DouSwip;
        break;
    case GESTURE_CW_DOUBLELINE:
        gesture_upload = DouSwip;
        break;
    case GESTURE_M:
        gesture_upload = Mgesture;
        break;
    case GESTURE_W:
        gesture_upload = Wgesture;
        break;
    }
    if(ret_event)
    {

        input_report_key(private_ts->input_dev, KEY_F4, 1);
        input_sync(private_ts->input_dev);

        input_report_key(private_ts->input_dev, KEY_F4, 0);
        input_sync(private_ts->input_dev);
        //FAKE_POWER_KEY_SEND=true;

    }
    private_ts->gesture_cust_en[0x30] = 1;
    private_ts->gesture_cust_en[0x57] = 1;
    private_ts->gesture_cust_en[0x31] = 1;
    private_ts->gesture_cust_en[0x32] = 1;
    private_ts->gesture_cust_en[0x54] = 1;
    private_ts->gesture_cust_en[0x51] = 1;
    private_ts->gesture_cust_en[0x52] = 1;
    private_ts->gesture_cust_en[0x24] = 1;
    private_ts->gesture_cust_en[0x28] = 1;
    private_ts->gesture_cust_en[0x27] = 1;
}

int himax_gesture_coordinate_show(struct seq_file *m, void *v)
{
    size_t ret = 0;
    E("enter::%s\n",__func__);
    ret = seq_printf(m, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", gesture_upload,
                 coordinate_report[0], coordinate_report[1], coordinate_report[2], coordinate_report[3],
                 coordinate_report[4], coordinate_report[5], coordinate_report[6], coordinate_report[7],
                 coordinate_report[8], coordinate_report[9], coordinate_report[10], coordinate_report[11],
                 coordinate_report[12]);
    return ret;
}

#endif

int himax_report_data_init(void)
{
    if(hx_touch_data->hx_coord_buf!=NULL)
    {
        kfree(hx_touch_data->hx_coord_buf);
    }
#ifdef HX_TP_PROC_DIAG
    if(hx_touch_data->hx_rawdata_buf!=NULL)
    {
        kfree(hx_touch_data->hx_rawdata_buf);
    }
#endif

#if defined(HX_SMART_WAKEUP)
    hx_touch_data->event_size = himax_get_touch_data_size();
    if(hx_touch_data->hx_event_buf!=NULL)
    {
        kfree(hx_touch_data->hx_event_buf);
    }
#endif

    hx_touch_data->touch_all_size = himax_get_touch_data_size();
    hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT/4;
    hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT%4;

    if (hx_touch_data->raw_cnt_rmd != 0x00) //more than 4 fingers
    {
        hx_touch_data->rawdata_size = cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
        hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT+hx_touch_data->raw_cnt_max+2)*4;
    }
    else //less than 4 fingers
    {
        hx_touch_data->rawdata_size = cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
        hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT+hx_touch_data->raw_cnt_max+1)*4;
    }
    if((ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) % hx_touch_data->rawdata_size == 0)
        hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size;
    else
        hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size + 1;
    I("%s: rawdata_frame_size = %d ",__func__,hx_touch_data->rawdata_frame_size);
    I("%s: ic_data->HX_MAX_PT:%d,hx_raw_cnt_max:%d,hx_raw_cnt_rmd:%d,g_hx_rawdata_size:%d,hx_touch_data->touch_info_size:%d\n",__func__,ic_data->HX_MAX_PT,hx_touch_data->raw_cnt_max,hx_touch_data->raw_cnt_rmd,hx_touch_data->rawdata_size,hx_touch_data->touch_info_size);

    hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t)*(hx_touch_data->touch_info_size),GFP_KERNEL);
    if(hx_touch_data->hx_coord_buf == NULL)
        goto mem_alloc_fail;
#ifdef HX_TP_PROC_DIAG
    hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t)*(hx_touch_data->touch_all_size - hx_touch_data->touch_info_size),GFP_KERNEL);
    if(hx_touch_data->hx_rawdata_buf == NULL)
        goto mem_alloc_fail;
#endif

#if defined(HX_SMART_WAKEUP)
    hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t)*(hx_touch_data->event_size),GFP_KERNEL);
    if(hx_touch_data->hx_event_buf == NULL)
        goto mem_alloc_fail;
#endif

    return NO_ERR;

mem_alloc_fail:
    kfree(hx_touch_data->hx_coord_buf);
#if defined(HX_TP_PROC_DIAG)
    kfree(hx_touch_data->hx_rawdata_buf);
#endif
#if defined(HX_SMART_WAKEUP)
    kfree(hx_touch_data->hx_event_buf);
#endif

    I("%s: Memory allocate fail!\n",__func__);
    return MEM_ALLOC_FAIL;

}


#if defined(HX_USB_DETECT_GLOBAL)
void himax_cable_detect_func(bool force_renew)
{
    struct himax_ts_data *ts;
    u32 connect_status = 0;

    connect_status = USB_detect_flag;//upmu_is_chr_det();
    ts = private_ts;
    //I("Touch: cable status=%d, cable_config=%p, usb_connected=%d \n", connect_status,ts->cable_config, ts->usb_connected);
    if (ts->cable_config)
    {
        if (((!!connect_status) != ts->usb_connected) || force_renew)
        {

            if (!!connect_status)
            {
                ts->cable_config[1] = 0x01;
                ts->usb_connected = 0x01;
            }
            else
            {
                ts->cable_config[1] = 0x00;
                ts->usb_connected = 0x00;
            }

            himax_usb_detect_set(ts->client,ts->cable_config);

            I("%s: Cable status change: 0x%2.2X\n", __func__, ts->usb_connected);
        }
        //else
        //I("%s: Cable status is the same as previous one, ignore.\n", __func__);
    }
}
#endif

static void himax_report_points(struct himax_ts_data *ts)
{
    int x = 0;
    int y = 0;
    int w = 0;
    int base = 0;
    int32_t	loop_i = 0;
    uint16_t old_finger = 0;

    //I("%s:Entering\n",__func__);

    /* finger on/press */
    if (hx_point_num != 0 )
    {
        old_finger = ts->pre_finger_mask;
        ts->pre_finger_mask = 0;
        hx_touch_data->finger_num = hx_touch_data->hx_coord_buf[ts->coordInfoSize - 4] & 0x0F;
        hx_touch_data->finger_on = 1;
        AA_press = 1;
        for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++)
        {
            base = loop_i * 4;
            x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
            y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
            w = hx_touch_data->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];
            //x = ic_data->HX_X_RES - x;
            //y = ic_data->HX_Y_RES - y;
            if(x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 && y <= ts->pdata->abs_y_max)
            {
                hx_touch_data->finger_num--;
                if ((ts->debug_log_level & BIT(3)) > 0)
                {
                    himax_log_touch_event_detail(ts,x,y,w,loop_i,EN_NoiseFilter,HX_FINGER_ON,old_finger);
                }
#ifndef HX_PROTOCOL_A

                input_mt_slot(ts->input_dev, loop_i);
#endif
                input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
#ifndef HX_PROTOCOL_A
                input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
                input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
#endif
                input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

#ifndef HX_PROTOCOL_A
                ts->last_slot = loop_i;
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
#else
                input_mt_sync(ts->input_dev);
#endif

                if (!ts->first_pressed)
                {
                    ts->first_pressed = 1;
                    I("S1@%d, %d\n", x, y);
                }

                ts->pre_finger_data[loop_i][0] = x;
                ts->pre_finger_data[loop_i][1] = y;
                if (ts->debug_log_level & BIT(1))
                    himax_log_touch_event(x, y, w,loop_i,EN_NoiseFilter,HX_FINGER_ON);

                ts->pre_finger_mask = ts->pre_finger_mask + (1 << loop_i);
            }
            /* report coordinates */
            else
            {
#ifndef HX_PROTOCOL_A
                input_mt_slot(ts->input_dev, loop_i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif

                if (loop_i == 0 && ts->first_pressed == 1)
                {
                    ts->first_pressed = 2;
                    I("E1@%d, %d\n",
                      ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
                }
                if ((ts->debug_log_level & BIT(3)) > 0)
                {
                    himax_log_touch_event_detail(ts,x,y,w,loop_i,Last_EN_NoiseFilter,HX_FINGER_LEAVE,old_finger);
                }
            }
        }
#ifndef HX_PROTOCOL_A
        input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
#endif
        input_sync(ts->input_dev);
    }

    /* finger leave/release */
    else
    {
#if defined(HX_PALM_REPORT)
        if(himax_palm_detect(hx_touch_data->hx_coord_buf) == NO_ERR)
        {
            I(" %s HX_PALM_REPORT KEY power event press\n",__func__);
            input_report_key(ts->input_dev, KEY_POWER, 1);
            input_sync(ts->input_dev);
            msleep(100);
            I(" %s HX_PALM_REPORT KEY power event release\n",__func__);
            input_report_key(ts->input_dev, KEY_POWER, 0);
            input_sync(ts->input_dev);
            return;
        }
#endif
        hx_touch_data->finger_on = 0;
        AA_press = 0;

        for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++)
        {
            if (((ts->pre_finger_mask >> loop_i) & 1) == 1)
            {
#ifndef HX_PROTOCOL_A
                input_mt_slot(ts->input_dev, loop_i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
            }
            if(ts->pre_finger_mask > 0 && (ts->debug_log_level & BIT(3)) > 0)
            {
                if (((ts->pre_finger_mask >> loop_i) & 1) == 1)
                {
                    if (ts->useScreenRes)
                    {
                        I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
                          ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
                    }
                    else
                    {
                        I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",0, loop_i+1, ts->pre_finger_data[loop_i][0],ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
                    }
                }
            }
        }
        if (ts->pre_finger_mask > 0)
        {
            ts->pre_finger_mask = 0;
        }

        if (ts->first_pressed == 1)
        {
            ts->first_pressed = 2;
            I("E1@%d, %d\n",ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
        }

        if (ts->debug_log_level & BIT(1))
            himax_log_touch_event(x, y, w,loop_i,EN_NoiseFilter,HX_FINGER_LEAVE);

        input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
        input_sync(ts->input_dev);
    }
    Last_EN_NoiseFilter = EN_NoiseFilter;

    //I("%s:End\n",__func__);
}

static void himax_report_all_leave(struct himax_ts_data *ts)
{
    int loop_i = 0;

    for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++)
    {
#ifndef HX_PROTOCOL_A
        input_mt_slot(ts->input_dev, loop_i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
    }
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_sync(ts->input_dev);
}

int himax_touch_get(struct himax_ts_data *ts,uint8_t *buf,int ts_status)
{
    int ret = 0;

    switch(ts_status)
    {
    /*normal*/
    case 1:
#ifdef HX_TP_PROC_DIAG
        hx_touch_data->diag_cmd = getDiagCommand();

        if((hx_touch_data->diag_cmd)
                || (HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
                || (HX_ESD_RESET_ACTIVATE)
#endif
          )
        {
            ret = himax_read_event_stack(ts->client, buf, 128);
        }
        else
        {
            ret = himax_read_event_stack(ts->client, buf, hx_touch_data->touch_info_size);
        }

        if (!ret)
#else
        if(!himax_read_event_stack(ts->client, buf, hx_touch_data->touch_info_size))
#endif
        {
            E("%s: can't read data from chip!\n", __func__);
            goto err_workqueue_out;
        }
        break;
#if defined(HX_SMART_WAKEUP)
    /*SMWP*/
    case 2:
        himax_burst_enable(ts->client, 0);
        if(!himax_read_event_stack(ts->client,buf,hx_touch_data->event_size))
        {
            E("%s: can't read data from chip!\n", __func__);
            goto err_workqueue_out;
        }
        break;
#endif
    default:
        break;
    }
    return NO_ERR;

err_workqueue_out:
    return I2C_FAIL;

}

int himax_checksum_cal(struct himax_ts_data *ts,uint8_t *buf,int ts_status)
{
#if defined(HX_ESD_RECOVERY)
    int hx_EB_event = 0;
    int hx_EC_event = 0;
    int hx_ED_event = 0;
    int hx_esd_event = 0;
    int hx_zero_event = 0;
    int shaking_ret = 0;
#endif
    uint16_t check_sum_cal = 0;
    int32_t	loop_i = 0;
    int length = 0;

    /* Normal */
    if(ts_status == HX_REPORT_COORD)
        length = hx_touch_data->touch_info_size;
#if defined(HX_SMART_WAKEUP)
    /* SMWP */
    else if(ts_status == HX_REPORT_SMWP_EVENT)
        length = (GEST_PTLG_ID_LEN+GEST_PTLG_HDR_LEN);
#endif
    else
    {
        I("%s, Neither Normal Nor SMWP error!\n",__func__);
    }
    //I("Now status=%d,length=%d\n",ts_status,length);
    for (loop_i = 0; loop_i < length; loop_i++)
    {
        check_sum_cal+=buf[loop_i];

#ifdef HX_ESD_RECOVERY
        if(ts_status == HX_REPORT_COORD)
        {
            /* case 1 ESD recovery flow */
            if(buf[loop_i] == 0xEB)
                hx_EB_event++;
            else if(buf[loop_i] == 0xEC)
                hx_EC_event++;
            else if(buf[loop_i] == 0xED)
                hx_ED_event++;
            /* case 2 ESD recovery flow-Disable */
            else if(buf[loop_i] == 0x00)
                hx_zero_event++;
            else
            {
                hx_EB_event = 0;
                hx_EC_event = 0;
                hx_ED_event = 0;
                hx_zero_event = 0;
                g_zero_event_count = 0;
            }

            if(hx_EB_event == length)
            {
                hx_esd_event = length;
                hx_EB_event_flag ++;
                I("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
            }
            else if(hx_EC_event == length)
            {
                hx_esd_event = length;
                hx_EC_event_flag ++;
                I("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
            }
            else if(hx_ED_event == length)
            {
                hx_esd_event = length;
                hx_ED_event_flag ++;
                I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
            }
            else
            {
                hx_esd_event = 0;
            }
        }
#endif
    }

    if(ts_status == HX_REPORT_COORD)
    {
#ifdef HX_ESD_RECOVERY
        if ((hx_esd_event == length || hx_zero_event == length)
                && (HX_HW_RESET_ACTIVATE == 0)
                && (HX_ESD_RESET_ACTIVATE == 0)
#if defined(HX_TP_PROC_DIAG)
                && (hx_touch_data->diag_cmd == 0)
#endif
#if defined(HX_TP_PROC_SELF_TEST) || defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
                && (g_self_test_entered == 0)
#endif
           )
        {
            shaking_ret = himax_ic_esd_recovery(hx_esd_event,hx_zero_event,length);
            if(shaking_ret == CHECKSUM_FAIL)
            {
                himax_esd_hw_reset();
                goto checksum_fail;
            }
            else if(shaking_ret == ERR_WORK_OUT)
                goto err_workqueue_out;
            else
            {
                //I("I2C running. Nothing to be done!\n");
                goto workqueue_out;
            }
        }
        else if (HX_ESD_RESET_ACTIVATE)
        {
#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
#ifdef HX_RESUME_SEND_CMD
            himax_rst_cmd_recovery_func(ts->suspended);
#endif
#endif
            /* drop 1st interrupts after chip reset */
            HX_ESD_RESET_ACTIVATE = 0;
            I("[HX_ESD_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
            goto checksum_fail;
        }

        else if (HX_HW_RESET_ACTIVATE)
#else
        if (HX_HW_RESET_ACTIVATE)
#endif
        {
#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
#ifdef HX_RESUME_SEND_CMD
            himax_rst_cmd_recovery_func(ts->suspended);
#endif
#endif
            /* drop 1st interrupts after chip reset */
            HX_HW_RESET_ACTIVATE = 0;
            I("[HX_HW_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
            goto ready_to_serve;
        }
    }

    if ((check_sum_cal % 0x100 != 0) )
    {
        I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
        goto checksum_fail;
    }

    /* I("%s:End\n",__func__); */
    return NO_ERR;

ready_to_serve:
    return READY_TO_SERVE;
checksum_fail:
    return CHECKSUM_FAIL;
#ifdef HX_ESD_RECOVERY
err_workqueue_out:
    return ERR_WORK_OUT;
workqueue_out:
    return WORK_OUT;
#endif
}

int himax_ts_work_status(struct himax_ts_data *ts)
{
    /* 1: normal, 2:SMWP */
    int result = HX_REPORT_COORD;
    uint8_t diag_cmd = 0;

#ifdef HX_TP_PROC_DIAG
    diag_cmd = getDiagCommand();
#endif

#ifdef HX_SMART_WAKEUP
    //if (atomic_read(&ts->suspend_mode)&&(!FAKE_POWER_KEY_SEND)&&(ts->SMWP_enable)&&(!diag_cmd))
    if (atomic_read(&ts->suspend_mode) && (ts->SMWP_enable) && (!diag_cmd))
    {
        result = HX_REPORT_SMWP_EVENT;
    }
#endif
    /* I("Now Status is %d\n",result); */
    return result;
}

void himax_assign_touch_data(uint8_t *buf,int ts_status)
{
    uint8_t hx_state_info_pos = hx_touch_data->touch_info_size - 3;

    if(ts_status == HX_REPORT_COORD)
    {
        memcpy(hx_touch_data->hx_coord_buf,&buf[0],hx_touch_data->touch_info_size);
        if(buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF )
            memcpy(hx_touch_data->hx_state_info,&buf[hx_state_info_pos],2);
        else
            memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
    }
#if defined(HX_SMART_WAKEUP)
    else
        memcpy(hx_touch_data->hx_event_buf,buf,hx_touch_data->event_size);
#endif

#ifdef HX_TP_PROC_DIAG
    if((hx_touch_data->diag_cmd)
            || (HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
            || (HX_ESD_RESET_ACTIVATE)
#endif
      )
    {
        memcpy(hx_touch_data->hx_rawdata_buf,&buf[hx_touch_data->touch_info_size],hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
    }
#endif

}

void himax_coord_report(struct himax_ts_data *ts)
{

#if defined(HX_TP_PROC_DIAG)
    //touch monitor raw data fetch
    if(himax_set_diag_cmd(ic_data,hx_touch_data))
        I("%s: coordinate dump fail and bypass with checksum err\n",__func__);
#endif
    EN_NoiseFilter = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT+2]>>3);
    EN_NoiseFilter = EN_NoiseFilter & 0x01;
    tpd_key = 0x00;
    p_point_num = hx_point_num;
    if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
        hx_point_num = 0;
    else
        hx_point_num= hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

    /* Touch Point information */
    if(!tpd_key && !tpd_key_old)
        himax_report_points(ts);
}

void himax_ts_work(struct himax_ts_data *ts)
{
    uint8_t hw_reset_check[2];
    uint8_t buf[128];
    int check_sum_cal = 0;
    //int loop_i = 0;
    int ts_status = 0;

#if defined(HX_USB_DETECT_GLOBAL)
    himax_cable_detect_func(false);
#endif

    ts_status = himax_ts_work_status(ts);
    if(ts_status > HX_REPORT_SMWP_EVENT || ts_status < HX_REPORT_COORD)
        goto neither_normal_nor_smwp;

    memset(buf, 0x00, sizeof(buf));
    memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

    //I("New Method for ts_work\n");

    if(himax_touch_get(ts,buf,ts_status))
        goto err_workqueue_out;

    if (ts->debug_log_level & BIT(0))
    {
        himax_log_touch_data(buf,hx_touch_data);
    }

    check_sum_cal = himax_checksum_cal(ts,buf,ts_status);
    if (check_sum_cal == CHECKSUM_FAIL)
        goto checksum_fail;
    else if (check_sum_cal == READY_TO_SERVE)
        goto ready_to_serve;
    else if (check_sum_cal == ERR_WORK_OUT)
        goto err_workqueue_out;
    else if (check_sum_cal == WORK_OUT)
        goto workqueue_out;
    /* checksum calculate pass and assign data to global touch data*/
    else
        himax_assign_touch_data(buf,ts_status);

    if(ts_status == HX_REPORT_COORD)
        himax_coord_report(ts);
#if defined(HX_SMART_WAKEUP)
    else
    {
        wake_lock_timeout(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
        himax_wake_check_func();
    }
#endif

checksum_fail:
workqueue_out:
ready_to_serve:
neither_normal_nor_smwp:
    return;

err_workqueue_out:
    I("%s: Now reset the Touch chip.\n", __func__);

#ifdef HX_RST_PIN_FUNC
    himax_ic_reset(false,true);
#endif
    goto workqueue_out;
}
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
    struct himax_ts_data *ts;

    ts = container_of(timer, struct himax_ts_data, timer);
    queue_work(ts->himax_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

#if defined( HX_USB_DETECT_CALLBACK)
static void himax_cable_tp_status_handler_func(int connect_status)
{
    struct himax_ts_data *ts;
    I("Touch: cable change to %d\n", connect_status);
    ts = private_ts;
    if (ts->cable_config)
    {
        if (!atomic_read(&ts->suspend_mode))
        {
            if ((!!connect_status) != ts->usb_connected)
            {
                if (!!connect_status)
                {
                    ts->cable_config[1] = 0x01;
                    ts->usb_connected = 0x01;
                }
                else
                {
                    ts->cable_config[1] = 0x00;
                    ts->usb_connected = 0x00;
                }

                i2c_himax_master_write(ts->client, ts->cable_config,
                                       sizeof(ts->cable_config), DEFAULT_RETRY_CNT);

                I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
            }
            else
                I("%s: Cable status is the same as previous one, ignore.\n", __func__);
        }
        else
        {
            if (connect_status)
                ts->usb_connected = 0x01;
            else
                ts->usb_connected = 0x00;
            I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
        }
    }
}

static struct t_cable_status_notifier himax_cable_status_handler =
{
    .name = "usb_tp_connected",
    .func = himax_cable_tp_status_handler_func,
};

#endif

#ifdef HX_AUTO_UPDATE_FW
static void himax_update_register(struct work_struct *work)
{
    I(" %s in\n", __func__);
    if(i_update_FW() == false)
        I("NOT Have new FW=NOT UPDATE=\n");
    else
        I("Have new FW=UPDATE=\n");
    I(" %s out\n", __func__);
}
#endif

#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
    int ret = 0;
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
                                            work_att.work);
    I(" %s in\n", __func__);

    ts->fb_notif.notifier_call = fb_notifier_callback;
    // [yanlin] Reduce screen on time
    //ret = fb_register_client(&ts->fb_notif);
    if (ret)
        E(" Unable to register fb_notifier: %d\n", ret);
}
#endif

#ifdef  HX_TP_PROC_FLASH_DUMP
static void himax_ts_flash_work_func(struct work_struct *work)
{
    himax_ts_flash_func();
}
#endif


#ifdef  HX_TP_PROC_DIAG
static void himax_ts_diag_work_func(struct work_struct *work)
{
    himax_ts_diag_func();
}
#endif

int himax_chip_common_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef HX_AUTO_UPDATE_FW
    bool auto_update_flag = false;
#endif
    int ret = 0, err = 0;
    struct himax_ts_data *ts;
    struct himax_i2c_platform_data *pdata;

    //Check I2C functionality
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        E("%s: i2c check functionality error\n", __func__);
        err = -ENODEV;
        goto err_check_functionality_failed;
    }

    pr_err("tp913::client->addr=0x%x\n",client->addr);
    if (client->addr != 0x48){
        client->addr = 0x48;
        pr_err("tp913::after modify:i2c_client->addr=0x%x\n",client->addr);
    }
    atomic_set(&double_enable, 0);//[lichenggang start]add for gesture
    atomic_set(&close_incall,0);//[lichenggang start]add for psensor disable when call

    ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        E("%s: allocate himax_ts_data failed\n", __func__);
        err = -ENOMEM;
        goto err_alloc_data_failed;
    }

    i2c_set_clientdata(client, ts);
    ts->client = client;
    ts->dev = &client->dev;
    mutex_init(&ts->rw_lock);

    pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
    if (pdata == NULL)   /*Allocate Platform data space*/
    {
        err = -ENOMEM;
        goto err_dt_platform_data_fail;
    }

    ic_data = kzalloc(sizeof(*ic_data), GFP_KERNEL);
    if (ic_data == NULL)   /*Allocate IC data space*/
    {
        err = -ENOMEM;
        goto err_dt_ic_data_fail;
    }

    /* allocate report data */
    hx_touch_data = kzalloc(sizeof(struct himax_report_data),GFP_KERNEL);
    if(hx_touch_data == NULL)
    {
        err = -ENOMEM;
        goto err_alloc_touch_data_failed;
    }

    if (himax_parse_dt(ts, pdata) < 0)
    {
        I(" pdata is NULL for DT\n");
        goto err_alloc_dt_pdata_failed;
    }

#ifdef HX_RST_PIN_FUNC
    ts->rst_gpio = pdata->gpio_reset;
#endif

    himax_gpio_power_config(ts->client, pdata);

#ifndef CONFIG_OF
    if (pdata->power)
    {
        ret = pdata->power(1);
        if (ret < 0)
        {
            E("%s: power on failed\n", __func__);
            goto err_power_failed;
        }
    }
#endif
    private_ts = ts;
    if (himax_ic_package_check(ts->client) == false)
    {
        E("Himax chip doesn NOT EXIST");
        goto err_ic_package_failed;
    }

    if (pdata->virtual_key)
        ts->button = pdata->virtual_key;
#ifdef  HX_TP_PROC_FLASH_DUMP
    ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
    if (!ts->flash_wq)
    {
        E("%s: create flash workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_flash_dump_wq_failed;
    }

    INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);

    setSysOperation(0);
    setFlashBuffer();
#endif


#ifdef  HX_TP_PROC_DIAG
    ts->himax_diag_wq = create_singlethread_workqueue("himax_diag");
    if (!ts->himax_diag_wq)
    {
        E("%s: create diag workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_diag_wq_failed;
    }
    INIT_DELAYED_WORK(&ts->himax_diag_delay_wrok, himax_ts_diag_work_func);
#endif

/* [lichenggang start] modify for tp firmware update */
    boot_mode = get_boot_mode();
    pr_err("INIT::boot_mode = %d ............\n",boot_mode);
    if(boot_mode == NORMAL_BOOT){
        #ifdef HX_AUTO_UPDATE_FW
            auto_update_flag = himax_calculateChecksum(client, false);
            auto_update_flag |= himax_flash_lastdata_check(client);
            if(auto_update_flag)
                goto FW_force_upgrade;
        #endif
            himax_read_FW_ver(client);
        #ifdef HX_AUTO_UPDATE_FW
        FW_force_upgrade:
            E("firmware infor: vendor_fw_ver=0x%x\n", ic_data->vendor_fw_ver);
            E("firmware infor: g_i_FW_VER=0x%x\n", g_i_FW_VER);
            E("firmware infor: vendor_config_ver=0x%x\n", ic_data->vendor_config_ver);
            E("firmware infor: g_i_CFG_VER=0x%x\n", g_i_CFG_VER);

            auto_update_flag |= (( ic_data->vendor_fw_ver < g_i_FW_VER ) || ( ic_data->vendor_config_ver < g_i_CFG_VER ));
            /* Not sure to do */
            //auto_update_flag |= ((ic_data->vendor_cid_maj_ver != g_i_CID_MAJ) || (ic_data->vendor_cid_min_ver < g_i_CID_MIN));
            if (auto_update_flag)
            {
                ts->himax_update_wq = create_singlethread_workqueue("HMX_update_reuqest");
                if (!ts->himax_update_wq)
                {
                    E(" allocate syn_update_wq failed\n");
                    err = -ENOMEM;
                    goto err_update_wq_failed;
                }
                INIT_DELAYED_WORK(&ts->work_update, himax_update_register);
                queue_delayed_work(ts->himax_update_wq, &ts->work_update, msecs_to_jiffies(2000));
            }
        #endif
    }
/* [lichenggang end] */

#ifdef HX_ZERO_FLASH
    ts->himax_0f_update_wq = create_singlethread_workqueue("HMX_0f_update_reuqest");
    if (!ts->himax_0f_update_wq) {
    E(" allocate himax_0f_update_wq failed\n");
    err = -ENOMEM;
    goto err_0f_update_wq_failed;
    }
    INIT_DELAYED_WORK(&ts->work_0f_update, himax_0f_operation);
    queue_delayed_work(ts->himax_0f_update_wq, &ts->work_0f_update, msecs_to_jiffies(2000));
#endif
    //Himax Power On and Load Config
    if (himax_loadSensorConfig(client, pdata))
    {
        E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
        goto err_detect_failed;
    }
    himax_power_on_init(client);

    calculate_point_number();
#ifdef HX_TP_PROC_DIAG
    setXChannel(ic_data->HX_RX_NUM); // X channel
    setYChannel(ic_data->HX_TX_NUM); // Y channel

    setMutualBuffer();
    /* [yanlin start] TP threshold test */
    setSelfMaxBuffer();//morgen
    setMutualMaxBuffer();
    /* [yanlin end] */
    setMutualNewBuffer();
    setMutualOldBuffer();
    if (getMutualBuffer() == NULL)
    {
        E("%s: mutual buffer allocate fail failed\n", __func__);
        return -1;
    }
#ifdef HX_TP_PROC_2T2R
    if(Is_2T2R)
    {
        setXChannel_2(ic_data->HX_RX_NUM_2); // X channel
        setYChannel_2(ic_data->HX_TX_NUM_2); // Y channel

        setMutualBuffer_2();

        if (getMutualBuffer_2() == NULL)
        {
            E("%s: mutual buffer 2 allocate fail failed\n", __func__);
            return -1;
        }
    }
#endif
#endif
#ifdef CONFIG_OF
    ts->power = pdata->power;
#endif
    ts->pdata = pdata;

    ts->x_channel = ic_data->HX_RX_NUM;
    ts->y_channel = ic_data->HX_TX_NUM;
    ts->nFinger_support = ic_data->HX_MAX_PT;
    //calculate the i2c data size
    calcDataSize(ts->nFinger_support);
    I("%s: calcDataSize complete\n", __func__);
#ifdef CONFIG_OF
    ts->pdata->abs_pressure_min        = 0;
    ts->pdata->abs_pressure_max        = 200;
    ts->pdata->abs_width_min           = 0;
    ts->pdata->abs_width_max           = 200;
    pdata->cable_config[0]             = 0xF0;
    pdata->cable_config[1]             = 0x00;
#endif
    ts->suspended                      = false;
#if defined( HX_USB_DETECT_CALLBACK)||defined(HX_USB_DETECT_GLOBAL)
    ts->usb_connected = 0x00;
    ts->cable_config = pdata->cable_config;
#endif

#ifdef HX_PROTOCOL_A
    ts->protocol_type = PROTOCOL_TYPE_A;
#else
    ts->protocol_type = PROTOCOL_TYPE_B;
#endif
    I("%s: Use Protocol Type %c\n", __func__,
      ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

    ret = himax_input_register(ts);
    if (ret)
    {
        E("%s: Unable to register %s input device\n",
          __func__, ts->input_dev->name);
        goto err_input_register_device_failed;
    }

#ifdef CONFIG_FB
    ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");
    if (!ts->himax_att_wq)
    {
        E(" allocate syn_att_wq failed\n");
        err = -ENOMEM;
        goto err_get_intr_bit_failed;
    }
    INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
    queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));
#endif


#ifdef HX_SMART_WAKEUP
    ts->SMWP_enable=0;
    wake_lock_init(&ts->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX_common_NAME);
#endif

#ifdef HX_HIGH_SENSE
    ts->HSEN_enable=0;
#endif

/* [lichenggang start] add for sensitivity adjustment */
#ifdef HX_THRESHOLD_ADJUST_ENABLE
    sensitivity_addr[3] = 0x10;
    sensitivity_addr[2] = 0x00;
    sensitivity_addr[1] = 0x70;
    sensitivity_addr[0] = 0x98;

    sensitivity_addr1[3] = 0x10;
    sensitivity_addr1[2] = 0x00;
    sensitivity_addr1[1] = 0x70;
    sensitivity_addr1[0] = 0x9c;

    himax_register_read(client, sensitivity_addr, 4, sensitivity_data, false);
    himax_register_read(client, sensitivity_addr1, 4, sensitivity_data1, false);

    I("data[0] = 0x%2x, data[1] = 0x%2x, data[2] = 0x%2x, data[3] = 0x%2x\n", sensitivity_data[0], sensitivity_data[1], sensitivity_data[2], sensitivity_data[3]);
    I("data1[0] = 0x%2x, data1[1] = 0x%2x, data1[2] = 0x%2x, data1[3] = 0x%2x\n", sensitivity_data1[0], sensitivity_data1[1], sensitivity_data1[2], sensitivity_data1[3]);
    //ts->sensitivity = system_sensitivity;//0:low,1:normal,2:high

    ts->sensitivity = 1;//0:low,1:normal,2:high
    himax_set_sensitivity_enable(ts->client, ts->sensitivity);
#endif
/* [lichenggang end] */

/* [lichenggang start] add for gesture , support all ID default */
    ts->gesture_cust_en[0x30] = 1;
    ts->gesture_cust_en[0x57] = 1;
    ts->gesture_cust_en[0x31] = 1;
    ts->gesture_cust_en[0x32] = 1;
    ts->gesture_cust_en[0x54] = 1;
    ts->gesture_cust_en[0x51] = 1;
    ts->gesture_cust_en[0x52] = 1;
    ts->gesture_cust_en[0x24] = 1;
    ts->gesture_cust_en[0x28] = 1;
    ts->gesture_cust_en[0x27] = 1;
/* [lichenggang end] */

    //touch data init
    err=himax_report_data_init();
    if(err)
        goto err_report_data_init_failed;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
    himax_touch_proc_init();
#endif
    himax_touch_tp_info_init();//[lichenggang]add for tp_info node

#if defined( HX_USB_DETECT_CALLBACK)
    if (ts->cable_config)
        cable_detect_register_notifier(&himax_cable_status_handler);
#endif

    err = himax_ts_register_interrupt(ts->client);
    if (err)
        goto err_register_interrupt_failed;

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
    if (auto_update_flag)
    {
        himax_int_enable(client->irq,0);
    }
#endif
    return 0;

err_register_interrupt_failed:
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
    himax_touch_proc_deinit();
#endif
err_report_data_init_failed:

#ifdef HX_SMART_WAKEUP
    wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef CONFIG_FB
    cancel_delayed_work_sync(&ts->work_att);
    destroy_workqueue(ts->himax_att_wq);
    err_get_intr_bit_failed:
#endif
    err_input_register_device_failed:
    input_free_device(ts->input_dev);
err_detect_failed:
#ifdef HX_ZERO_FLASH
    cancel_delayed_work_sync(&ts->work_0f_update);
    destroy_workqueue(ts->himax_0f_update_wq);
err_0f_update_wq_failed:
#endif
#ifdef HX_AUTO_UPDATE_FW
    if (auto_update_flag) {
        cancel_delayed_work_sync(&ts->work_update);
        destroy_workqueue(ts->himax_update_wq);
    }
err_update_wq_failed:
#endif

#ifdef  HX_TP_PROC_DIAG
    cancel_delayed_work_sync(&ts->himax_diag_delay_wrok);
    destroy_workqueue(ts->himax_diag_wq);
    err_create_diag_wq_failed:
#endif


#ifdef  HX_TP_PROC_FLASH_DUMP
    destroy_workqueue(ts->flash_wq);
    err_create_flash_dump_wq_failed:
#endif
err_ic_package_failed:

    if (gpio_is_valid(pdata->gpio_irq))
        gpio_free(pdata->gpio_irq);
#ifdef HX_RST_PIN_FUNC
    if (gpio_is_valid(pdata->gpio_reset))
        gpio_free(pdata->gpio_reset);
#endif

#ifndef CONFIG_OF
err_power_failed:
#endif

err_alloc_dt_pdata_failed:
    kfree(hx_touch_data);
err_alloc_touch_data_failed:
    kfree(ic_data);
err_dt_ic_data_fail:
    kfree(pdata);

err_dt_platform_data_fail:
    kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
    probe_fail_flag = 1;
    return err;

}

int himax_chip_common_remove(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);

if (!ts->use_irq)
{
    hrtimer_cancel(&ts->timer);
    destroy_workqueue(ts->himax_wq);
}

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
    himax_touch_proc_deinit();
#endif

#ifdef HX_SMART_WAKEUP
    wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef CONFIG_FB
    if (fb_unregister_client(&ts->fb_notif))
        E("Error occurred while unregistering fb_notifier.\n");
    cancel_delayed_work_sync(&ts->work_att);
    destroy_workqueue(ts->himax_att_wq);
#endif
    input_free_device(ts->input_dev);
#ifdef HX_ZERO_FLASH
    cancel_delayed_work_sync(&ts->work_0f_update);
    destroy_workqueue(ts->himax_0f_update_wq);
#endif
#ifdef HX_AUTO_UPDATE_FW
    cancel_delayed_work_sync(&ts->work_update);
    destroy_workqueue(ts->himax_update_wq);
#endif

#ifdef HX_TP_PROC_DIAG
    cancel_delayed_work_sync(&ts->himax_diag_delay_wrok);
    destroy_workqueue(ts->himax_diag_wq);
#endif


#ifdef HX_TP_PROC_FLASH_DUMP
    destroy_workqueue(ts->flash_wq);
#endif
    if (gpio_is_valid(ts->pdata->gpio_irq))
        gpio_free(ts->pdata->gpio_irq);
#ifdef HX_RST_PIN_FUNC
    if (gpio_is_valid(ts->pdata->gpio_reset))
        gpio_free(ts->pdata->gpio_reset);
#endif

    kfree(hx_touch_data);
    kfree(ic_data);
    kfree(ts->pdata);
    kfree(ts);
    probe_fail_flag = 0;
  return 0;
}

int himax_chip_common_suspend(struct himax_ts_data *ts)
{
    int ret;

    //atomic_set(&close_incall,0);

    if(ts->suspended)
    {
        I("%s: Already suspended. Skipped. \n", __func__);
        return 0;
    }
    else
    {
        ts->suspended = true;
        I("%s: enter \n", __func__);
    }

#ifdef HX_TP_PROC_FLASH_DUMP
    if (getFlashDumpGoing())
    {
        I("[himax] %s: Flash dump is going, reject suspend\n",__func__);
        return 0;
    }
#endif


#ifdef HX_SMART_WAKEUP
    ts->gesture_cust_en[0x30] = 1;
    ts->gesture_cust_en[0x57] = 1;
    ts->gesture_cust_en[0x31] = 1;
    ts->gesture_cust_en[0x32] = 1;
    ts->gesture_cust_en[0x54] = 1;
    ts->gesture_cust_en[0x51] = 1;
    ts->gesture_cust_en[0x52] = 1;
    ts->gesture_cust_en[0x24] = 1;
    ts->gesture_cust_en[0x28] = 1;
    ts->gesture_cust_en[0x27] = 1;
    if(ts->SMWP_enable)
    {
        himax_set_SMWP_enable(ts->client,ts->SMWP_enable,ts->suspended);/* [lichenggang] modify for driver logic err */
        atomic_set(&ts->suspend_mode, 1);
        ts->pre_finger_mask = 0;
        //FAKE_POWER_KEY_SEND=false;
        I("[himax] %s: SMART_WAKEUP enable, reject suspend\n",__func__);
        return 0;
    }
#endif

    himax_int_enable(ts->client->irq,0);
    himax_suspend_ic_action(ts->client);
    himax_rst_gpio_set(private_ts->rst_gpio, 0);/* [lichenggang] modify for driver logic err */
    if (!ts->use_irq)
    {
        ret = cancel_work_sync(&ts->work);
        if (ret)
            himax_int_enable(ts->client->irq,1);
    }

    //ts->first_pressed = 0;
    atomic_set(&ts->suspend_mode, 1);
    ts->pre_finger_mask = 0;
    if (ts->pdata->powerOff3V3 && ts->pdata->power)
        ts->pdata->power(0);
    I("%s: END \n", __func__);
    return 0;
}

int himax_chip_common_resume(struct himax_ts_data *ts)
{

    I("%s: enter \n", __func__);
    atomic_set(&close_incall,0);
    //atomic_set(&double_enable,0);

    if(ts->suspended == false)
    {
        I("%s: It had entered resume,skip this step \n", __func__);
        return 0;
    }
    else
        ts->suspended = false;

    atomic_set(&ts->suspend_mode, 0);
    if (ts->pdata->powerOff3V3 && ts->pdata->power){
        ts->pdata->power(1);
    }

    /* [lichenggang start] add for gesture , support all ID default */
    ts->gesture_cust_en[0x30] = 1;
    ts->gesture_cust_en[0x57] = 1;
    ts->gesture_cust_en[0x31] = 1;
    ts->gesture_cust_en[0x32] = 1;
    ts->gesture_cust_en[0x54] = 1;
    ts->gesture_cust_en[0x51] = 1;
    ts->gesture_cust_en[0x52] = 1;
    ts->gesture_cust_en[0x24] = 1;
    ts->gesture_cust_en[0x28] = 1;
    ts->gesture_cust_en[0x27] = 1;
    /* [lichenggang end] */


#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)||defined(HX_THRESHOLD_ADJUST_ENABLE)
    himax_resend_cmd_func(ts->suspended);
#elif defined(HX_RESUME_HW_RESET)
    himax_ic_reset(false,false);
#endif
    himax_report_all_leave(ts);
    himax_resume_ic_action(ts->client);
    himax_int_enable(ts->client->irq,1);
    I("%s: END \n", __func__);
    return 0;
}

