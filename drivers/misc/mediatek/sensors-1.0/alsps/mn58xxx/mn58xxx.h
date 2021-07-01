#ifndef __ELAN_MN58xxx__
#define __ELAN_MN58xxx__

#define ELAN_IOCTL_MAGIC 'c'
#define ELAN_EPL8800_IOCTL_GET_PFLAG _IOR(ELAN_IOCTL_MAGIC, 1, int *)
#define ELAN_EPL8800_IOCTL_GET_LFLAG _IOR(ELAN_IOCTL_MAGIC, 2, int *)
#define ELAN_EPL8800_IOCTL_ENABLE_PFLAG _IOW(ELAN_IOCTL_MAGIC, 3, int *)
#define ELAN_EPL8800_IOCTL_ENABLE_LFLAG _IOW(ELAN_IOCTL_MAGIC, 4, int *)
#define ELAN_EPL8800_IOCTL_GETDATA _IOR(ELAN_IOCTL_MAGIC, 5, int *)

#define MODE_IDLE			(0)
#define MODE_ALS			(1)
#define MODE_PS				(2)
#define MODE_PS_ALS		    (5)

#define EPL_MODE_IDLE		(0x00)
#define EPL_MODE_ALS		(0x01)
#define EPL_MODE_PS			(0x02)
#define EPL_MODE_ALS_PS		(0x03)

#define POWER_DOWN		    (1)
#define POWER_WAKE			(0)

#define RESET				(0<<1)
#define RUN					(1<<1)

#define ALS_SIGLE_DIS       (0<<2)
#define ALS_SIGLE_EN        (1<<2)

#define EPL_ALS_INTT_2			(0<<2)
#define EPL_ALS_INTT_4			(1<<2)
#define EPL_ALS_INTT_8			(2<<2)
#define EPL_ALS_INTT_16			(3<<2)
#define EPL_ALS_INTT_32			(4<<2)
#define EPL_ALS_INTT_64			(5<<2)
#define EPL_ALS_INTT_128		(6<<2)
#define EPL_ALS_INTT_256		(7<<2)
#define EPL_ALS_INTT_512		(8<<2)
#define EPL_ALS_INTT_768		(9<<2)
#define EPL_ALS_INTT_1024		(10<<2)
#define EPL_ALS_INTT_2048		(11<<2)
#define EPL_ALS_INTT_4096		(12<<2)
#define EPL_ALS_INTT_6144		(13<<2)
#define EPL_ALS_INTT_8192		(14<<2)
#define EPL_ALS_INTT_10240		(15<<2)
static int als_intt_value[] = {2, 4, 8, 16, 32, 64, 128, 256, 512, 768, 1024, 2048, 4096, 6144, 8192, 10240};

#define EPL_RS_0			(0<<5)
#define EPL_RS_2			(1<<5)
#define EPL_RS_4			(2<<5)
#define EPL_RS_8			(3<<5)
#define EPL_RS_16			(4<<5)
#define EPL_RS_32			(5<<5)
#define EPL_RS_64		    (6<<5)
#define EPL_RS_128		    (7<<5)

#define EPL_PS_INTT_4			(0<<2)
#define EPL_PS_INTT_8			(1<<2)
#define EPL_PS_INTT_16			(2<<2)
#define EPL_PS_INTT_24			(3<<2)
#define EPL_PS_INTT_32			(4<<2)
#define EPL_PS_INTT_48			(5<<2)
#define EPL_PS_INTT_80			(6<<2)
#define EPL_PS_INTT_144			(7<<2)
#define EPL_PS_INTT_272			(8<<2)
#define EPL_PS_INTT_384			(9<<2)
#define EPL_PS_INTT_520			(10<<2)
#define EPL_PS_INTT_784			(11<<2)
#define EPL_PS_INTT_1040		(12<<2)
#define EPL_PS_INTT_2064		(13<<2)
#define EPL_PS_INTT_4112		(14<<2)
#define EPL_PS_INTT_6160		(15<<2)
static int ps_intt_value[] = {4, 8, 16, 24, 32, 48, 80, 144, 272, 384, 520, 784, 1040, 2064, 4112, 6160};

#define EPL_WAIT_0_MS			(0x0<<4)
#define EPL_WAIT_2_MS			(0x1<<4)
#define EPL_WAIT_4_MS			(0x2<<4)
#define EPL_WAIT_8_MS			(0x3<<4)
#define EPL_WAIT_12_MS			(0x4<<4)
#define EPL_WAIT_20_MS			(0x5<<4)
#define EPL_WAIT_30_MS			(0x6<<4)
#define EPL_WAIT_40_MS			(0x7<<4)
#define EPL_WAIT_50_MS			(0x8<<4)
#define EPL_WAIT_75_MS			(0x9<<4)
#define EPL_WAIT_100_MS		    (0xA<<4)
#define EPL_WAIT_150_MS		    (0xB<<4)
#define EPL_WAIT_200_MS		    (0xC<<4)
#define EPL_WAIT_300_MS		    (0xD<<4)
#define EPL_WAIT_400_MS		    (0xE<<4)
#define EPL_WAIT_SINGLE		    (0x0F << 4)

static int wait_value[] = {0, 2, 4, 8, 12, 20, 30, 40, 50, 75, 100, 150, 200, 300, 400};
int wait_len = sizeof(wait_value)/sizeof(int);

#define EPL_AG_EN	        (0x00)
#define EPL_AG_L            (1<<4)
#define EPL_AG_M            (0<<4)

#define EPL_GAIN_HIGH		(0x00)
#define EPL_GAIN_MID		(0x01)
#define EPL_GAIN_LOW		(0x03)

#define EPL_PSALS_OSR_11	(0x00 << 3)
#define EPL_PSALS_OSR_12	(0x01 << 3)
#define EPL_PSALS_OSR_13	(0x02 << 3)
#define EPL_PSALS_OSR_14	(0x03 << 3)
static int osr_value[] = {128, 256, 512, 1024};

#define EPL_CYCLE_1			(0x00)
#define EPL_CYCLE_2			(0x01)
#define EPL_CYCLE_4			(0x02)
#define EPL_CYCLE_8			(0x03)
#define EPL_CYCLE_16		(0x04)
#define EPL_CYCLE_32		(0x05)
#define EPL_CYCLE_64		(0x06)
static int cycle_value[] = {1, 2, 4, 8, 16, 32, 64};

#define EPL_T_EN         1
#define EPL_T_DIS        0

#define EPL_PS_PRE          (0 << 6)
#define EPL_PS_STD          (1 << 6)

#define EPL_ALS_PRE          (0 << 5)
#define EPL_ALS_STD          (1 << 5)

#define EPL_IR_ON_CTRL_OFF	(0x00 << 5)
#define EPL_IR_ON_CTRL_ON	(0x01 << 5)

#define EPL_IR_MODE_CURRENT	(0x00 << 4)
#define EPL_IR_MODE_VOLTAGE	(0x01 << 4)

#define EPL_IR_DRIVE_200	(0x00)
#define EPL_IR_DRIVE_100	(0x01)
#define EPL_IR_DRIVE_50		(0x02)
#define EPL_IR_DRIVE_10		(0x03)


#define EPL_INT_CTRL_ALS_OR_PS		(0x00 << 4)
#define EPL_INT_CTRL_ALS			(0x01 << 4)
#define EPL_INT_CTRL_PS				(0x02 << 4)

#define EPL_PERIST_1		(0x00 << 2)
#define EPL_PERIST_4		(0x01 << 2)
#define EPL_PERIST_8		(0x02 << 2)
#define EPL_PERIST_16		(0x03 << 2)

#define EPL_INTTY_DISABLE	(0x00)
#define EPL_INTTY_BINARY	(0x01)
#define EPL_INTTY_ACTIVE	(0x02)
#define EPL_INTTY_FRAME	    (0x03)

#define EPL_RESETN_RESET	(0x00 << 1)
#define EPL_RESETN_RUN		(0x01 << 1)

#define EPL_POWER_OFF		(0x01)
#define EPL_POWER_ON		(0x00)

#define EPL_ALS_INT_CHSEL_0	(0x00 << 4)
#define EPL_ALS_INT_CHSEL_1	(0x01 << 4)

#define EPL_SATURATION				(0x01 << 5)
#define EPL_SATURATION_NOT		    (0x00 << 5)

#define EPL_CMP_H_TRIGGER		(0x01 << 4)
#define EPL_CMP_H_CLEAR		    (0x00 << 4)

#define EPL_CMP_L_TRIGGER		(0x01 << 3)
#define EPL_CMP_L_CLEAR		    (0x00 << 3)

#define EPL_INT_TRIGGER		(0x01 << 2)
#define EPL_INT_CLEAR		(0x00 << 2)

#define EPL_CMP_RESET		(0x00 << 1)
#define EPL_CMP_RUN			(0x01 << 1)

#define EPL_LOCK			(0x01)
#define EPL_UN_LOCK		    (0x00)

#define EPL_OSC_SEL_1MHZ	(0x07)

#define EPL_REVNO           (0x81)

#define EPL_A_D             (0x7 << 4)
#define EPL_INTERNAL        (0xb << 4)

#define EPL_NORMAL      (0 << 3)
#define EPL_BYBASS      (1 << 3)

#define EPL_GFIN_DISABLE      (0 << 2)
#define EPL_GFIN_ENABLE       (1 << 2)

#define EPL_VOS_DISABLE      (0 << 1)
#define EPL_VOS_ENABLE       (1 << 1)

#define EPL_DOC_OFF         (0)
#define EPL_DOC_ON          (1)

struct _ps_data
{
	u16 ir_data;
	u16 data;
};

struct _ps_factory
{
	bool calibration_enable;
	bool calibrated;
	u16 cancelation;
	u16 high_threshold;
	u16 low_threshold;
};

#define ALS_CHANNEL_SIZE	2
struct _als_data
{
	u16 channels[ALS_CHANNEL_SIZE];
	u16 lux;
};

struct _als_factory
{
	bool calibration_enable;
	bool calibrated;
	u16 lux_per_count;
};

struct _ps_setting
{
    bool polling_mode;
	u8 integration_time;
	u8 gain;
	u8 osr;
	u8 cycle;
    u8 ps_rs;
	u8 ps_intb_nonlos;
	u16 high_threshold;
	u16 low_threshold;
	u8 ps_std;
	u8 ir_on_control;
	u8 ir_mode;
	u8 ir_drive;
	u8 persist;
	u8 interrupt_type;
	u8 saturation_1;
	u8 saturation;
	u8 compare_high;
	u8 compare_low;
	u8 interrupt_flag;
	u8 compare_reset;
	u8 lock;
	u16 cancelation;
	struct _ps_data data;
	struct _ps_factory factory;
};

struct _als_setting
{
    bool polling_mode;
	u8 report_type;
	u16 report_count;
	u8 integration_time;
	u8 als_rs;
	u8 als_intb_nonlos;
	u8 gain;
    u8 osr;
	u8 cycle;
	u16 high_threshold;
	u16 low_threshold;
	u8 als_std;
	u8 persist;
	u8 interrupt_type;
	u8 saturation;
	u8 compare_high;
	u8 compare_low;
	u8 interrupt_flag;
	u8 compare_reset;
	u8 lock;
	u8 interrupt_channel_select;
	u16 dyn_intt_raw;
	//auto gain
	u8 als_ag_l;
	u8 als_ag_h;
	u8 als_aintt_l;
	u8 als_aintt_h;
	u16 als_ag_thd;
    u8 als_ag_l_value;
    u8 als_ag_h_value;
    u8 als_ag_flag;
    u8 als_ag_rs_weight;
	struct _als_data data;
	struct _als_factory factory;
};

typedef struct _sensor
{
	u8 wait;
	u8 mode;
	u8 als_single_pixel;
    bool enable_factory_calibration;
	u8 early_suspend_mode;
	u8 osc_sel;
	u8 interrupt_control;
	u8 reset;
	u8 power;
	struct _ps_setting ps;
	struct _als_setting als;
	u16 revno;
}epl_optical_sensor;

typedef enum {
    PS_NEAR,
    PS_FAR
}ps_report_status_t;

/*REG Table*/
#define    DEVREG_ENABLE        0x00 //REG0x00
#define    DEVREG_ALS_CONFIG    0x01 //REG0x01
#define    DEVREG_ALS_FILT      0x02 //REG0x02
#define    DEVREG_PS_CONFIG     0x03 //REG0x03
#define    DEVREG_PS_FILT       0x04 //REG0x04
#define    DEVREG_LED_CONFIG    0x05 //REG0x05
#define    DEVREG_PS_INT        0x06 //REG0x06
#define    DEVREG_ALS_INT       0x07 //REG0x07
#define    DEVREG_ALS_ILTL      0x08 //REG0x08
#define    DEVREG_ALS_ILTH      0x09 //REG0x09
#define    DEVREG_ALS_IHTL      0x0A //REG0x0A
#define    DEVREG_ALS_IHTH      0x0B //REG0x0B
#define    DEVREG_PS_ILTL       0x0C //REG0x0C
#define    DEVREG_PS_ILTH       0x0D //REG0x0D
#define    DEVREG_PS_IHTL       0x0E //REG0x0E
#define    DEVREG_PS_IHTH       0x0F //REG0x0F
#define    DEVREG_CONTROL       0x10 //REG0x10
#define    DEVREG_RESET         0x11 //REG0x11
#define    DEVREG_ALS_STATUS    0x12 //REG0x12
#define    DEVREG_C0DATAL       0x13 //REG0x13
#define    DEVREG_C0DATAH       0x14 //REG0x14
#define    DEVREG_C1DATAL       0x15 //REG0x15
#define    DEVREG_C1DATAH       0x16 //REG0x16
#define    DEVREG_PS_STATUS     0x1B //REG0x1B
#define    DEVREG_PS_ADATAL     0x1C //REG0x1C
#define    DEVREG_PS_ADATAH     0x1D //REG0x1D
#define    DEVREG_PS_RDATAL     0x1E //REG0x1E
#define    DEVREG_PS_RDATAH     0x1F //REG0x1F
#define    DEVREG_REV_ID        0x20 //REG0x20
#define    DEVREG_CHIP_ID       0x21 //REG0x21 not defined in spec
#define    DEVREG_PS_OFSL       0x22 //REG0x22
#define    DEVREG_PS_OFSH       0x23 //REG0x23

#endif
