// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for SmartSense SC910GS camera.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* --------------------------------------------------------------------------
 * Registers / limits
 * --------------------------------------------------------------------------
 */
#define SC910GS_STREAM_DELAY_US          25000
#define SC910GS_STREAM_DELAY_RANGE_US    1000

/* Initialisation delay after reset deassertion. */
#define SC910GS_XCLR_MIN_DELAY_US        10000
#define SC910GS_XCLR_DELAY_RANGE_US      1000
#define SC910GS_XCLR_ASSERTED            1
#define SC910GS_XCLR_DEASSERTED          0

/* V4L2 exposure is in line units; the SC910GS register uses 2-line units. */
#define SC910GS_REG_EXPOSURE             CCI_REG24(0x3e00)
#define SC910GS_EXPOSURE_MIN             0
#define SC910GS_EXPOSURE_STEP            2
#define SC910GS_EXPOSURE_DEFAULT         500
#define SC910GS_EXPOSURE_MARGIN          8
#define SC910GS_EXPOSURE_MAX(vmax)       ((vmax) - SC910GS_EXPOSURE_MARGIN)
#define SC910GS_EXPOSURE_TO_REG(exposure) ((exposure) / 2)


#define SC910GS_REG_VMAX           CCI_REG16(0x320e) //Technicall not called VMAX but I'm used to that naming
#define SC910GS_VMAX_DEFAULT       2500


#define SC910GS_REG_ID             CCI_REG16(0x3107)
#define SC910GS_ID                 0xa22b

#define SC910GS_REG_MODE_SELECT       CCI_REG8(0x0100)
#define SC910GS_MODE_STANDBY          0x00
#define SC910GS_MODE_STREAMING        0x01
#define SC910GS_REG_RESET             CCI_REG8(0x0103)

/* Black level control */
#define SC910GS_REG_BLKLEVEL             CCI_REG16(0x3907)
#define SC910GS_BLKLEVEL_DEFAULT         100

/* Analog gain control */
#define SC910GS_REG_ANALOG_GAIN          CCI_REG8(0x3E08)
#define SC910GS_REG_ANALOG_GAIN_FINE     CCI_REG8(0x3E09)
#define SC910GS_REG_AGC_CTRL             CCI_REG8(0x3E03)
#define SC910GS_AGC_CTRL_MASK            GENMASK(3, 0)
#define SC910GS_AGC_CTRL_MANUAL          0x0b
#define SC910GS_ANA_GAIN_MIN             0
#define SC910GS_ANA_GAIN_MAX             348
#define SC910GS_ANA_GAIN_STEP            1
#define SC910GS_ANA_GAIN_DEFAULT         0

#define SC910GS_REG_TEST_PATTERN         CCI_REG8(0x4501)
#define SC910GS_TEST_PATTERN_DISABLED    0xb4
#define SC910GS_TEST_PATTERN_INCREMENTAL 0xbc

/* Vertical and Horizontal Flip */
#define SC910GS_REG_FLIP            CCI_REG8(0x3221)
#define SC910GS_HFLIP_MASK          GENMASK(2, 1)
#define SC910GS_VFLIP_MASK          GENMASK(7, 5)

/* Pixel rate used by the V4L2 frame-duration model for this 30 fps mode. */
#define SC910GS_PIXEL_RATE              288000000U
#define SC910GS_LINK_FREQ               495000000U

#define SC910GS_XCLK_MIN                10000000U
#define SC910GS_XCLK_MAX                60000000U

static const s64 sc910gs_link_freq_menu[] = {
    SC910GS_LINK_FREQ,
};


/* Full pixel array is 3856x2368; the programmed mode outputs a centered 4K crop. */
#define SC910GS_NATIVE_WIDTH       3856U
#define SC910GS_NATIVE_HEIGHT      2368U
#define SC910GS_PIXEL_ARRAY_LEFT      0U
#define SC910GS_PIXEL_ARRAY_TOP       0U
#define SC910GS_PIXEL_ARRAY_WIDTH  SC910GS_NATIVE_WIDTH
#define SC910GS_PIXEL_ARRAY_HEIGHT SC910GS_NATIVE_HEIGHT
#define SC910GS_MODE_WIDTH         3840U
#define SC910GS_MODE_HEIGHT        2160U
#define SC910GS_MODE_CROP_LEFT        8U
#define SC910GS_MODE_CROP_TOP       104U


static const struct cci_reg_sequence mode_common_regs[] = {
	{CCI_REG8(0x36e9),0x80},
	{CCI_REG8(0x37f9),0x80},
	{CCI_REG8(0x36ea),0x0b},
	{CCI_REG8(0x36eb),0x0b},
	{CCI_REG8(0x36ec),0x03},
	{CCI_REG8(0x36ed),0x21},
	{CCI_REG8(0x37fa),0x0a},
	{CCI_REG8(0x37fb),0x32},
	{CCI_REG8(0x37fc),0x10},
	{CCI_REG8(0x37fd),0x34},
	{CCI_REG8(0x36e9),0x44},
	{CCI_REG8(0x37f9),0x40},
	{CCI_REG8(0x3018),0x72}, //4lanemode
	{CCI_REG8(0x3019),0xf0},
	{CCI_REG8(0x301f),0x8f},
	{CCI_REG8(0x3031),0x0c}, //raw12mode
	{CCI_REG8(0x3033),0xa2},
	
	{CCI_REG8(0x3106),0x01},
	{CCI_REG8(0x3200),0x00},
	{CCI_REG8(0x3201),0x00},
	{CCI_REG8(0x3202),0x00},
	{CCI_REG8(0x3203),0x64},
	{CCI_REG8(0x3204),0x0f},
	{CCI_REG8(0x3205),0x0f},
	{CCI_REG8(0x3206),0x08},
	{CCI_REG8(0x3207),0xdb},
	{CCI_REG8(0x3208),0x0f},
	{CCI_REG8(0x3209),0x00}, //width=3840 
	{CCI_REG8(0x320a),0x08},
	{CCI_REG8(0x320b),0x70}, //height=2160
	{CCI_REG8(0x320c),0x02},
	{CCI_REG8(0x320d),0x58}, //HMAX
	{CCI_REG8(0x320e),0x09},
	{CCI_REG8(0x320f),0xc4}, //VMAX
	{CCI_REG8(0x3210),0x00},
	{CCI_REG8(0x3211),0x08},
	{CCI_REG8(0x3212),0x00},
    {CCI_REG8(0x3213),0x04},
	{CCI_REG8(0x3271),0x1b},
	{CCI_REG8(0x3273),0x1f},
	{CCI_REG8(0x3275),0x1b},
	{CCI_REG8(0x3277),0x1f},
	{CCI_REG8(0x3306),0x88},
	{CCI_REG8(0x3308),0x10},
	{CCI_REG8(0x330a),0x01},
	{CCI_REG8(0x330b),0x10},
	{CCI_REG8(0x3314),0xf0},
	{CCI_REG8(0x3315),0x20},
	{CCI_REG8(0x3317),0xb0},
	{CCI_REG8(0x331f),0x02},
	{CCI_REG8(0x3320),0xc1},
	{CCI_REG8(0x3323),0x02},
	{CCI_REG8(0x3328),0xfb},
	{CCI_REG8(0x3364),0x0a},
	{CCI_REG8(0x3366),0x04},
	{CCI_REG8(0x3385),0x25},
	{CCI_REG8(0x3387),0x6d},
	{CCI_REG8(0x33ef),0x05},
	{CCI_REG8(0x33f8),0x02},
	{CCI_REG8(0x33fa),0x00},
	{CCI_REG8(0x3410),0xb0},
    
    {CCI_REG8(0x34f0),0x00}, //LED Strobe - all on

    {CCI_REG8(0x34f2),0x0f},
	{CCI_REG8(0x3630),0xa4},
	{CCI_REG8(0x3637),0x0f},
	{CCI_REG8(0x363b),0x08},
	{CCI_REG8(0x363c),0x07},
	{CCI_REG8(0x363d),0x07},
	{CCI_REG8(0x363e),0x67},
	{CCI_REG8(0x363f),0x07},
	{CCI_REG8(0x3648),0x99},
	{CCI_REG8(0x364e),0x02},
	{CCI_REG8(0x3654),0x00},
	{CCI_REG8(0x365c),0x00},
	{CCI_REG8(0x3727),0x07},
	{CCI_REG8(0x372d),0x00},
	{CCI_REG8(0x3731),0x00},
	{CCI_REG8(0x3732),0x00},
	{CCI_REG8(0x3733),0x08},
	{CCI_REG8(0x3904),0x18},
	{CCI_REG8(0x3905),0x2c},
    {CCI_REG8(0x3907),0x00}, // Blacklevel {16’h3907[4:0],16’h3908}
    {CCI_REG8(0x3908),0x00}, // Blacklevel
	{CCI_REG8(0x391d),0x04},
	{CCI_REG8(0x391f),0x19},
	{CCI_REG8(0x3926),0x21},
	{CCI_REG8(0x3927),0x01},
	{CCI_REG8(0x3950),0x18},
	{CCI_REG8(0x3e01),0x4d},
	{CCI_REG8(0x3e02),0xe0},
	{CCI_REG8(0x3e03),0x0b},
	{CCI_REG8(0x3e06),0x00},
	{CCI_REG8(0x3e08),0x03},
	{CCI_REG8(0x3e09),0x40},
	{CCI_REG8(0x4350),0x00},
	{CCI_REG8(0x4351),0x00},
	{CCI_REG8(0x4353),0x37},
	{CCI_REG8(0x4356),0x12},
	{CCI_REG8(0x4361),0xb0},
	{CCI_REG8(0x4366),0x1e},
	{CCI_REG8(0x440e),0x02},
	{CCI_REG8(0x4501),0xb4},
	{CCI_REG8(0x4509),0x41},
	{CCI_REG8(0x450d),0x09},
	{CCI_REG8(0x4800),0x24},
	{CCI_REG8(0x4837),0x10},
	{CCI_REG8(0x4900),0x24},
	{CCI_REG8(0x4937),0x14},
	{CCI_REG8(0x5000),0x0e},
	{CCI_REG8(0x5799),0x00},
	{CCI_REG8(0x5928),0x03},
	{CCI_REG8(0x59e0),0xc8},
	{CCI_REG8(0x59e1),0x1c},
	{CCI_REG8(0x59e2),0x10},
	{CCI_REG8(0x59e3),0x08},
	{CCI_REG8(0x59e4),0x00},
	{CCI_REG8(0x59e5),0x10},
	{CCI_REG8(0x59e6),0x08},
	{CCI_REG8(0x59e7),0x00},
	{CCI_REG8(0x59e8),0x18},
	{CCI_REG8(0x59e9),0x0c},
	{CCI_REG8(0x59ea),0x04},
	{CCI_REG8(0x59eb),0x18},
	{CCI_REG8(0x59ec),0x0c},
	{CCI_REG8(0x59ed),0x04},
	{CCI_REG8(0x59ee),0xc8},
	{CCI_REG8(0x59ef),0x1c},
	{CCI_REG8(0x59f4),0x10},
	{CCI_REG8(0x59f5),0x08},
	{CCI_REG8(0x59f6),0x00},
	{CCI_REG8(0x59f7),0x10},
	{CCI_REG8(0x59f8),0x08},
	{CCI_REG8(0x59f9),0x00},
	{CCI_REG8(0x59fa),0x18},
	{CCI_REG8(0x59fb),0x0c},
	{CCI_REG8(0x59fc),0x04},
	{CCI_REG8(0x59fd),0x18},
	{CCI_REG8(0x59fe),0x0c},
	{CCI_REG8(0x59ff),0x04},
};


/* Mode description */
struct sc910gs_mode {
    unsigned int width;
    unsigned int height;
    struct v4l2_rect crop;
    struct {
        unsigned int num_of_regs;
        const struct cci_reg_sequence *regs;
    } reg_list;
};

static struct sc910gs_mode supported_modes_12bit[] = {
    {
        .width = SC910GS_MODE_WIDTH,
        .height = SC910GS_MODE_HEIGHT,
        .crop = {
            .left = SC910GS_MODE_CROP_LEFT,
            .top = SC910GS_MODE_CROP_TOP,
            .width = SC910GS_MODE_WIDTH,
            .height = SC910GS_MODE_HEIGHT,
        },
    },
};

static const char * const sc910gs_test_pattern_menu[] = {
    "Disabled",
    "Incremental",
};

/* Regulators */
static const char * const sc910gs_supply_name[] = {
    "vana", /* 2.8V analog */
    "vdig", /* 1.5V core   */
    "vddl", /* 1.8V I/O    */
};

#define SC910GS_NUM_SUPPLIES ARRAY_SIZE(sc910gs_supply_name)

/* --------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------
 */

struct sc910gs {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct device *dev;
    struct regmap *regmap;

    struct clk *xclk;

    struct gpio_desc *reset_gpio;
    struct regulator_bulk_data supplies[SC910GS_NUM_SUPPLIES];

    struct v4l2_ctrl_handler ctrl_handler;

    /* Controls */
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *gain;
    struct v4l2_ctrl *vflip;
    struct v4l2_ctrl *hflip;
    struct v4l2_ctrl *vblank;
    struct v4l2_ctrl *hblank;
    struct v4l2_ctrl *blacklevel;
    struct v4l2_ctrl *test_pattern;


    bool streaming;
};

/* Helpers */

static inline struct sc910gs *to_sc910gs(struct v4l2_subdev *sd)
{
    return container_of(sd, struct sc910gs, sd);
}

static inline void get_mode_table(struct sc910gs *sc910gs, unsigned int code,
                  const struct sc910gs_mode **mode_list,
                  unsigned int *num_modes)
{
    switch (code) {
    case MEDIA_BUS_FMT_SRGGB12_1X12:
    case MEDIA_BUS_FMT_SGRBG12_1X12:
    case MEDIA_BUS_FMT_SGBRG12_1X12:
    case MEDIA_BUS_FMT_SBGGR12_1X12:
        *mode_list = supported_modes_12bit;
        *num_modes = ARRAY_SIZE(supported_modes_12bit);
        break;
    default:
        *mode_list = NULL;
        *num_modes = 0;
    }
}

static u32 sc910gs_get_format_code(struct sc910gs *sc910gs, u32 code)
{
    bool hflip = sc910gs->hflip ? sc910gs->hflip->val : false;
    bool vflip = sc910gs->vflip ? sc910gs->vflip->val : false;

    if (hflip && vflip)
        return MEDIA_BUS_FMT_SRGGB12_1X12;
    if (hflip)
        return MEDIA_BUS_FMT_SGBRG12_1X12;
    if (vflip)
        return MEDIA_BUS_FMT_SGRBG12_1X12;

    return MEDIA_BUS_FMT_SBGGR12_1X12;
}

static inline void sc910gs_gain_idx_to_regs(u16 idx, u8 *ana_gain, u8 *ana_fine)
{
    static const u8 ana_tbl[] = { 0x23, 0x27, 0x2F, 0x3F };
    u16 seg, off;

    if (!ana_gain || !ana_fine)
        return;

    /* clamp to valid range */
    if (idx > SC910GS_ANA_GAIN_MAX)
        idx = SC910GS_ANA_GAIN_MAX;

    /* Segment 0: ANA=0x03, FINE 0x40..0x7F */
    if (idx < 64) {
        *ana_gain = 0x03;
        *ana_fine = 0x40 + idx;
        return;
    }
    idx -= 64;

    /* Segment 1: ANA=0x07, FINE 0x40..0x5C (29 entries) */
    if (idx < 29) {
        *ana_gain = 0x07;
        *ana_fine = 0x40 + idx; /* up to 0x5C */
        return;
    }
    idx -= 29;

    /* Segments 2..5: ANA in ana_tbl[], FINE 0x40..0x7F */
    seg = idx / 64;
    off = idx % 64;

    /* Safety clamp (shouldn’t trigger for idx<=348) */
    if (seg >= ARRAY_SIZE(ana_tbl)) {
        seg = ARRAY_SIZE(ana_tbl) - 1;
        off = 63;
    }

    *ana_gain = ana_tbl[seg];
    *ana_fine = 0x40 + off;
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

static int sc910gs_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct sc910gs *sc910gs = container_of(ctrl->handler, struct sc910gs, ctrl_handler);
    const struct sc910gs_mode *mode, *mode_list;
    struct v4l2_subdev_state *state;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int num_modes;
    int pm_ret, ret = 0;
    u32 vmax, exposure_max;


    state = v4l2_subdev_get_locked_active_state(&sc910gs->sd);
    fmt = v4l2_subdev_state_get_format(state, 0);

    get_mode_table(sc910gs, fmt->code, &mode_list, &num_modes);
    mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
                      fmt->width, fmt->height);

    if (ctrl->id == V4L2_CID_VBLANK && sc910gs->exposure) {
        vmax = ctrl->val + mode->height;
        exposure_max = SC910GS_EXPOSURE_MAX(vmax);
        __v4l2_ctrl_modify_range(sc910gs->exposure,
                     SC910GS_EXPOSURE_MIN, exposure_max,
                     SC910GS_EXPOSURE_STEP, SC910GS_EXPOSURE_DEFAULT);
    }

    if (ctrl->id == V4L2_CID_VFLIP || ctrl->id == V4L2_CID_HFLIP)
        fmt->code = sc910gs_get_format_code(sc910gs, fmt->code);

    /* Apply control only when powered (runtime active). */
    pm_ret = pm_runtime_get_if_in_use(sc910gs->dev);
    if (pm_ret <= 0)
        return 0;

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE: {
        dev_dbg(sc910gs->dev, "EXPOSURE=%u\n", ctrl->val);
        ret = cci_write(sc910gs->regmap, SC910GS_REG_EXPOSURE,
                SC910GS_EXPOSURE_TO_REG((u32)ctrl->val) << 4, NULL);
        break;
    }
    case V4L2_CID_ANALOGUE_GAIN: {
        u8 ana_gain, ana_fine = 0;
        dev_dbg(sc910gs->dev, "ANALOG_GAIN=%u\n", ctrl->val);
        sc910gs_gain_idx_to_regs(ctrl->val, &ana_gain, &ana_fine);
        cci_update_bits(sc910gs->regmap, SC910GS_REG_AGC_CTRL,
                SC910GS_AGC_CTRL_MASK, SC910GS_AGC_CTRL_MANUAL, &ret);
        cci_write(sc910gs->regmap, SC910GS_REG_ANALOG_GAIN, ana_gain, &ret);
        cci_write(sc910gs->regmap, SC910GS_REG_ANALOG_GAIN_FINE, ana_fine, &ret);
        if (ret)
            dev_err_ratelimited(sc910gs->dev, "Gain write failed (%d)\n", ret);
        break;
    }
    case V4L2_CID_VBLANK: {
        vmax = ctrl->val + mode->height;
        dev_dbg(sc910gs->dev, "VBLANK=%u -> VMAX=%u\n", ctrl->val, vmax);
        ret = cci_write(sc910gs->regmap, SC910GS_REG_VMAX, vmax, NULL);
        break;
    }
    case V4L2_CID_HBLANK: {
        break;
    }
    case V4L2_CID_VFLIP:
        dev_dbg(sc910gs->dev, "V4L2_CID_VFLIP=%u\n", ctrl->val);
        cci_update_bits(sc910gs->regmap, SC910GS_REG_FLIP, SC910GS_VFLIP_MASK, FIELD_PREP(SC910GS_VFLIP_MASK, 0x07*ctrl->val), &ret);
        break;
    case V4L2_CID_HFLIP:
        dev_dbg(sc910gs->dev, "V4L2_CID_HFLIP=%u\n", ctrl->val);
        cci_update_bits(sc910gs->regmap, SC910GS_REG_FLIP, SC910GS_HFLIP_MASK, FIELD_PREP(SC910GS_HFLIP_MASK, 0x03*ctrl->val), &ret);
        break;
    case V4L2_CID_BRIGHTNESS: {
        u16 blacklevel = min_t(u32, ctrl->val, 4095);
        ret = cci_write(sc910gs->regmap, SC910GS_REG_BLKLEVEL, blacklevel, NULL);
        break;
    }
    case V4L2_CID_TEST_PATTERN:
        ret = cci_write(sc910gs->regmap, SC910GS_REG_TEST_PATTERN,
                ctrl->val ? SC910GS_TEST_PATTERN_INCREMENTAL :
                SC910GS_TEST_PATTERN_DISABLED, NULL);
        break;
    default:
        dev_dbg(sc910gs->dev, "Unhandled ctrl %s: id=0x%x, val=0x%x\n",
            ctrl->name, ctrl->id, ctrl->val);
        break;
    }

    pm_runtime_mark_last_busy(sc910gs->dev);
    pm_runtime_put_autosuspend(sc910gs->dev);
    return ret;
}

static const struct v4l2_ctrl_ops sc910gs_ctrl_ops = {
    .s_ctrl = sc910gs_set_ctrl,
};


static int sc910gs_init_controls(struct sc910gs *sc910gs)
{
    struct v4l2_ctrl_handler *hdl = &sc910gs->ctrl_handler;
    struct v4l2_fwnode_device_properties props;
    const struct sc910gs_mode *mode = &supported_modes_12bit[0];
    u32 vblank_def = SC910GS_VMAX_DEFAULT - mode->height;
    u32 exposure_max = SC910GS_EXPOSURE_MAX(SC910GS_VMAX_DEFAULT);
    int ret;

    ret = v4l2_ctrl_handler_init(hdl, 18);

    /* Read-only */
    sc910gs->pixel_rate = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                           V4L2_CID_PIXEL_RATE,
                           SC910GS_PIXEL_RATE, SC910GS_PIXEL_RATE, 1, SC910GS_PIXEL_RATE);
    if (sc910gs->pixel_rate)
        sc910gs->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    sc910gs->link_freq =
        v4l2_ctrl_new_int_menu(hdl, &sc910gs_ctrl_ops, V4L2_CID_LINK_FREQ,
                       0, 0, sc910gs_link_freq_menu);
    if (sc910gs->link_freq)
        sc910gs->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    sc910gs->vblank = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                       V4L2_CID_VBLANK, vblank_def,
                       0xFFFF - mode->height, 1, vblank_def);

    sc910gs->hblank = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                       V4L2_CID_HBLANK, 0, 0, 1, 0);
    if (sc910gs->hblank)
        sc910gs->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    sc910gs->exposure = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                         V4L2_CID_EXPOSURE,
                         SC910GS_EXPOSURE_MIN, exposure_max,
                         SC910GS_EXPOSURE_STEP, SC910GS_EXPOSURE_DEFAULT);

    sc910gs->gain = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                     SC910GS_ANA_GAIN_MIN, SC910GS_ANA_GAIN_MAX,
                     SC910GS_ANA_GAIN_STEP, SC910GS_ANA_GAIN_DEFAULT);

    sc910gs->vflip = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                      V4L2_CID_VFLIP, 0, 1, 1, 0);

    sc910gs->hflip = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                      V4L2_CID_HFLIP, 0, 1, 1, 0);

    sc910gs->blacklevel = v4l2_ctrl_new_std(hdl, &sc910gs_ctrl_ops,
                           V4L2_CID_BRIGHTNESS, 0, 0x1000-1, 1,
                           SC910GS_BLKLEVEL_DEFAULT);

    sc910gs->test_pattern =
        v4l2_ctrl_new_std_menu_items(hdl, &sc910gs_ctrl_ops,
                         V4L2_CID_TEST_PATTERN,
                         ARRAY_SIZE(sc910gs_test_pattern_menu) - 1,
                         0, 0, sc910gs_test_pattern_menu);

    if (hdl->error) {
        ret = hdl->error;
        dev_err(sc910gs->dev, "control init failed (%d)\n", ret);
        goto err_free;
    }

    ret = v4l2_fwnode_device_parse(sc910gs->dev, &props);
    if (ret)
        goto err_free;

    ret = v4l2_ctrl_new_fwnode_properties(hdl, &sc910gs_ctrl_ops, &props);
    if (ret)
        goto err_free;

    sc910gs->sd.ctrl_handler = hdl;
    return 0;

err_free:
    v4l2_ctrl_handler_free(hdl);
    return ret;
}

static void sc910gs_free_controls(struct sc910gs *sc910gs)
{
    v4l2_ctrl_handler_free(sc910gs->sd.ctrl_handler);
}

/* --------------------------------------------------------------------------
 * Pad ops / formats
 * --------------------------------------------------------------------------
 */

static int sc910gs_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct sc910gs *sc910gs = to_sc910gs(sd);

    if (code->index)
        return -EINVAL;

    code->code = sc910gs_get_format_code(sc910gs, code->code);
    return 0;
}

static int sc910gs_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_frame_size_enum *fse)
{
    struct sc910gs *sc910gs = to_sc910gs(sd);
    const struct sc910gs_mode *mode_list;
    unsigned int num_modes;

    get_mode_table(sc910gs, fse->code, &mode_list, &num_modes);
    if (fse->index >= num_modes)
        return -EINVAL;
    if (fse->code != sc910gs_get_format_code(sc910gs, fse->code))
        return -EINVAL;

    fse->min_width  = mode_list[fse->index].width;
    fse->max_width  = fse->min_width;
    fse->min_height = mode_list[fse->index].height;
    fse->max_height = fse->min_height;

    return 0;
}

static int sc910gs_set_pad_format(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_format *fmt)
{
    struct sc910gs *sc910gs = to_sc910gs(sd);
    const struct sc910gs_mode *mode_list, *mode;
    unsigned int num_modes;
    struct v4l2_mbus_framefmt *format;
    struct v4l2_rect *crop;

    /* Normalize requested code to what we really support */
    fmt->format.code = sc910gs_get_format_code(sc910gs, fmt->format.code);

    get_mode_table(sc910gs, fmt->format.code, &mode_list, &num_modes);
    mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
                                  fmt->format.width, fmt->format.height);

    fmt->format.width        = mode->width;
    fmt->format.height       = mode->height;
    fmt->format.field        = V4L2_FIELD_NONE;
    fmt->format.colorspace   = V4L2_COLORSPACE_RAW;
    fmt->format.ycbcr_enc    = V4L2_YCBCR_ENC_601;
    fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    fmt->format.xfer_func    = V4L2_XFER_FUNC_NONE;

    /* Update TRY/ACTIVE format kept by the framework */
    format = v4l2_subdev_state_get_format(sd_state, 0);
    *format = fmt->format;

    /* Keep the crop in sync with the selected mode */
    crop = v4l2_subdev_state_get_crop(sd_state, 0);
    *crop = mode->crop;

    return 0;
}



/* --------------------------------------------------------------------------
 * Stream on/off
 * --------------------------------------------------------------------------
 */

static int sc910gs_enable_streams(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state, u32 pad,
                 u64 streams_mask)
{
    struct sc910gs *sc910gs = to_sc910gs(sd);
    const struct sc910gs_mode *mode_list, *mode;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;
    int ret = 0;

    if (pad || streams_mask != BIT_ULL(0))
        return -EINVAL;

    if (sc910gs->streaming)
        return -EBUSY;

    fmt = v4l2_subdev_state_get_format(state, pad);
    get_mode_table(sc910gs, fmt->code, &mode_list, &n_modes);
    mode = v4l2_find_nearest_size(mode_list, n_modes, width, height,
                      fmt->width, fmt->height);
    if (!mode)
        return -EINVAL;

    ret = pm_runtime_resume_and_get(sc910gs->dev);
    if (ret)
        return ret;

    /* Software reset, then program the sensor while in standby. */
    cci_write(sc910gs->regmap, SC910GS_REG_RESET, 0x01, &ret);
    cci_write(sc910gs->regmap, SC910GS_REG_MODE_SELECT, SC910GS_MODE_STANDBY, &ret);
    if (ret)
        goto err_rpm_put;

    ret = cci_multi_reg_write(sc910gs->regmap, mode_common_regs,
                  ARRAY_SIZE(mode_common_regs), NULL);
    if (ret) {
        dev_err(sc910gs->dev, "Failed to write common settings\n");
        goto err_rpm_put;
    }

    usleep_range(50000, 51000);
    cci_write(sc910gs->regmap, CCI_REG8(0x331f), 0x12, &ret);
    cci_write(sc910gs->regmap, CCI_REG8(0x3385), 0x1d, &ret);
    usleep_range(50000, 51000);
    cci_write(sc910gs->regmap, CCI_REG8(0x331f), 0x02, &ret);
    cci_write(sc910gs->regmap, CCI_REG8(0x3385), 0x25, &ret);
    if (ret)
        goto err_rpm_put;

    /* Apply user controls after writing the base tables */
    ret = __v4l2_ctrl_handler_setup(sc910gs->sd.ctrl_handler);
    if (ret) {
        dev_err(sc910gs->dev, "Control handler setup failed\n");
        goto err_rpm_put;
    }

    ret = cci_write(sc910gs->regmap, SC910GS_REG_MODE_SELECT,
            SC910GS_MODE_STREAMING, NULL);
    if (ret)
        goto err_rpm_put;

    usleep_range(SC910GS_STREAM_DELAY_US,
             SC910GS_STREAM_DELAY_US + SC910GS_STREAM_DELAY_RANGE_US);

    /* vflip cannot change during streaming */
    __v4l2_ctrl_grab(sc910gs->vflip, true);
    __v4l2_ctrl_grab(sc910gs->hflip, true);
    sc910gs->streaming = true;

    return 0;

err_rpm_put:
    cci_write(sc910gs->regmap, SC910GS_REG_MODE_SELECT,
          SC910GS_MODE_STANDBY, NULL);
    pm_runtime_mark_last_busy(sc910gs->dev);
    pm_runtime_put_autosuspend(sc910gs->dev);
    return ret;
}

static int sc910gs_disable_streams(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *state, u32 pad,
                  u64 streams_mask)
{
    struct sc910gs *sc910gs = to_sc910gs(sd);
    int ret = 0;

    if (pad || streams_mask != BIT_ULL(0))
        return -EINVAL;

    if (!sc910gs->streaming)
        return 0;

    __v4l2_ctrl_grab(sc910gs->vflip, false);
    __v4l2_ctrl_grab(sc910gs->hflip, false);

    cci_write(sc910gs->regmap, SC910GS_REG_MODE_SELECT,
          SC910GS_MODE_STANDBY, &ret);
    sc910gs->streaming = false;

    pm_runtime_mark_last_busy(sc910gs->dev);
    pm_runtime_put_autosuspend(sc910gs->dev);

    return ret;
}

/* --------------------------------------------------------------------------
 * Power / runtime PM
 * --------------------------------------------------------------------------
 */

static int sc910gs_power_on(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct sc910gs *sc910gs = to_sc910gs(sd);
    int ret = 0;

    ret = regulator_bulk_enable(SC910GS_NUM_SUPPLIES, sc910gs->supplies);
    if (ret) {
        dev_err(sc910gs->dev, "Failed to enable regulators\n");
        return ret;
    }

    ret = clk_prepare_enable(sc910gs->xclk);
    if (ret) {
        dev_err(sc910gs->dev, "Failed to enable clock\n");
        goto reg_off;
    }

    gpiod_set_value_cansleep(sc910gs->reset_gpio, SC910GS_XCLR_DEASSERTED);

    usleep_range(SC910GS_XCLR_MIN_DELAY_US,
             SC910GS_XCLR_MIN_DELAY_US + SC910GS_XCLR_DELAY_RANGE_US);

    /* Software reset */
    cci_write(sc910gs->regmap, SC910GS_REG_RESET, 0x01, &ret);
    if (ret)
        goto clk_off;

    return 0;

clk_off:
    clk_disable_unprepare(sc910gs->xclk);
reg_off:
    regulator_bulk_disable(SC910GS_NUM_SUPPLIES, sc910gs->supplies);
    return ret;
}

static int sc910gs_power_off(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct sc910gs *sc910gs = to_sc910gs(sd);

    gpiod_set_value_cansleep(sc910gs->reset_gpio, SC910GS_XCLR_ASSERTED);
    regulator_bulk_disable(SC910GS_NUM_SUPPLIES, sc910gs->supplies);
    clk_disable_unprepare(sc910gs->xclk);

    return 0;
}

/* --------------------------------------------------------------------------
 * Selection / state
 * --------------------------------------------------------------------------
 */

static int sc910gs_get_selection(struct v4l2_subdev *sd,
                struct v4l2_subdev_state *sd_state,
                struct v4l2_subdev_selection *sel)
{
    switch (sel->target) {


    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.left   = 0;
        sel->r.top    = 0;
        sel->r.width  = SC910GS_NATIVE_WIDTH;   /* pixel array (no blanking) */
        sel->r.height = SC910GS_NATIVE_HEIGHT;
        return 0;
    case V4L2_SEL_TGT_CROP_BOUNDS:
        sel->r.top = SC910GS_PIXEL_ARRAY_TOP;
        sel->r.left = SC910GS_PIXEL_ARRAY_LEFT;
        sel->r.width = SC910GS_PIXEL_ARRAY_WIDTH;
        sel->r.height = SC910GS_PIXEL_ARRAY_HEIGHT;
        return 0;

    case V4L2_SEL_TGT_CROP_DEFAULT:
        sel->r = supported_modes_12bit[0].crop;
        return 0;

    case V4L2_SEL_TGT_CROP:
        sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
        return 0;

    default:
        return -EINVAL;
    }
}

static int sc910gs_init_state(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state)
{
    struct v4l2_rect *crop;
    struct v4l2_subdev_format fmt = {
        .which  = V4L2_SUBDEV_FORMAT_TRY,
        .pad    = 0,
        .format = {
            .code   = MEDIA_BUS_FMT_SBGGR12_1X12,
            .width  = SC910GS_MODE_WIDTH,
            .height = SC910GS_MODE_HEIGHT,
        },
    };

    sc910gs_set_pad_format(sd, state, &fmt);

    crop = v4l2_subdev_state_get_crop(state, 0);
    *crop = supported_modes_12bit[0].crop;

    return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */

static const struct v4l2_subdev_video_ops sc910gs_video_ops = {
    .s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc910gs_pad_ops = {
    .enum_mbus_code = sc910gs_enum_mbus_code,
    .get_fmt        = v4l2_subdev_get_fmt,
    .set_fmt        = sc910gs_set_pad_format,
    .get_selection  = sc910gs_get_selection,
    .enum_frame_size = sc910gs_enum_frame_size,
    .enable_streams  = sc910gs_enable_streams,
    .disable_streams = sc910gs_disable_streams,
};

static const struct v4l2_subdev_internal_ops sc910gs_internal_ops = {
    .init_state = sc910gs_init_state,
};

static const struct v4l2_subdev_core_ops sc910gs_core_ops = {
    .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
    .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops sc910gs_subdev_ops = {
    .core  = &sc910gs_core_ops,
    .video = &sc910gs_video_ops,
    .pad   = &sc910gs_pad_ops,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 * --------------------------------------------------------------------------
 */

static int sc910gs_check_hwcfg(struct device *dev, struct sc910gs *sc910gs)
{
    struct fwnode_handle *endpoint;
    struct v4l2_fwnode_endpoint ep = {
        .bus_type = V4L2_MBUS_CSI2_DPHY,
    };
    bool link_freq_found = false;
    unsigned int i;
    int ret = -EINVAL;

    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "endpoint node not found\n");
        return -EINVAL;
    }

    ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
    if (ret) {
        dev_err(dev, "could not parse endpoint\n");
        goto out_put;
    }

    if (ep.bus.mipi_csi2.num_data_lanes != 4) {
        dev_err(dev, "only 4 data lanes supported\n");
        goto out_free;
    }

    for (i = 0; i < ep.nr_of_link_frequencies; i++) {
        if (ep.link_frequencies[i] == SC910GS_LINK_FREQ) {
            link_freq_found = true;
            break;
        }
    }

    if (!link_freq_found) {
        dev_err(dev, "link frequency %u Hz not found in endpoint\n",
            SC910GS_LINK_FREQ);
        goto out_free;
    }

    ret = 0;

out_free:
    v4l2_fwnode_endpoint_free(&ep);
out_put:
    fwnode_handle_put(endpoint);
    return ret;
}

static int sc910gs_get_regulators(struct sc910gs *sc910gs)
{
    unsigned int i;

    for (i = 0; i < SC910GS_NUM_SUPPLIES; i++)
        sc910gs->supplies[i].supply = sc910gs_supply_name[i];

    return devm_regulator_bulk_get(sc910gs->dev,
                       SC910GS_NUM_SUPPLIES, sc910gs->supplies);
}

static int sc910gs_check_module_exists(struct sc910gs *sc910gs)
{
    int ret;
    u64 val;

    /* No chip-id register; read a known register as a presence test */
    ret = cci_read(sc910gs->regmap, SC910GS_REG_ID, &val, NULL);
    if (ret) {
        dev_err(sc910gs->dev, "register read failed (%d)\n", ret);
        return ret;
    }
    if (val != SC910GS_ID) {
        dev_err(sc910gs->dev, "ID mismatch, expected %x, actual %llx\n", SC910GS_ID, val);
        return 1;
    }

    dev_info(sc910gs->dev, "Sensor detected\n");
    return 0;
}

static int sc910gs_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct sc910gs *sc910gs;
    unsigned int xclk_freq;
    int ret;

    sc910gs = devm_kzalloc(dev, sizeof(*sc910gs), GFP_KERNEL);
    if (!sc910gs)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&sc910gs->sd, client, &sc910gs_subdev_ops);
    sc910gs->dev = dev;

    ret = sc910gs_check_hwcfg(dev, sc910gs);
    if (ret)
        return ret;

    sc910gs->regmap = devm_cci_regmap_init_i2c(client, 16);
    if (IS_ERR(sc910gs->regmap))
        return dev_err_probe(dev, PTR_ERR(sc910gs->regmap), "CCI init failed\n");

    sc910gs->xclk = devm_clk_get(dev, NULL);
    if (IS_ERR(sc910gs->xclk))
        return dev_err_probe(dev, PTR_ERR(sc910gs->xclk), "xclk missing\n");

    xclk_freq = clk_get_rate(sc910gs->xclk);
    if (xclk_freq < SC910GS_XCLK_MIN || xclk_freq > SC910GS_XCLK_MAX)
        return dev_err_probe(dev, -EINVAL,
                     "xclk frequency %u Hz out of range\n",
                     xclk_freq);

    ret = sc910gs_get_regulators(sc910gs);
    if (ret)
        return dev_err_probe(dev, ret, "regulators\n");

    sc910gs->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(sc910gs->reset_gpio))
        return dev_err_probe(dev, PTR_ERR(sc910gs->reset_gpio),
                     "failed to get reset GPIO\n");

    /* Power on to probe the device */
    ret = sc910gs_power_on(dev);
    if (ret)
        return ret;

    ret = sc910gs_check_module_exists(sc910gs);
    if (ret)
        goto err_power_off;

    pm_runtime_set_active(dev);
    pm_runtime_get_noresume(dev);
    pm_runtime_enable(dev);
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);

    ret = sc910gs_init_controls(sc910gs);
    if (ret)
        goto err_pm;

    sc910gs->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
                  V4L2_SUBDEV_FL_HAS_EVENTS;
    sc910gs->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    sc910gs->sd.internal_ops = &sc910gs_internal_ops;

    sc910gs->pad.flags = MEDIA_PAD_FL_SOURCE;

    ret = media_entity_pads_init(&sc910gs->sd.entity, 1, &sc910gs->pad);
    if (ret) {
        dev_err(dev, "entity pads init failed: %d\n", ret);
        goto err_ctrls;
    }

    sc910gs->sd.state_lock = sc910gs->ctrl_handler.lock;
    ret = v4l2_subdev_init_finalize(&sc910gs->sd);
    if (ret) {
        dev_err_probe(dev, ret, "subdev init\n");
        goto err_entity;
    }

    ret = v4l2_async_register_subdev_sensor(&sc910gs->sd);
    if (ret) {
        dev_err(dev, "sensor subdev register failed: %d\n", ret);
        goto err_subdev;
    }

    pm_runtime_mark_last_busy(dev);
    pm_runtime_put_autosuspend(dev);
    return 0;

err_subdev:
    v4l2_subdev_cleanup(&sc910gs->sd);
err_entity:
    media_entity_cleanup(&sc910gs->sd.entity);
err_ctrls:
    sc910gs_free_controls(sc910gs);
err_pm:
    pm_runtime_disable(dev);
    pm_runtime_set_suspended(dev);
err_power_off:
    sc910gs_power_off(dev);
    return ret;
}

static void sc910gs_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct sc910gs *sc910gs = to_sc910gs(sd);

    v4l2_async_unregister_subdev(sd);
    v4l2_subdev_cleanup(sd);
    media_entity_cleanup(&sd->entity);
    sc910gs_free_controls(sc910gs);

    pm_runtime_disable(sc910gs->dev);
    if (!pm_runtime_status_suspended(sc910gs->dev))
        sc910gs_power_off(sc910gs->dev);
    pm_runtime_set_suspended(sc910gs->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(sc910gs_pm_ops, sc910gs_power_off,
                 sc910gs_power_on, NULL);

static const struct of_device_id sc910gs_of_match[] = {
    { .compatible = "smartsens,sc910gs" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sc910gs_of_match);

static struct i2c_driver sc910gs_i2c_driver = {
    .driver = {
        .name  = "sc910gs",
        .pm    = pm_ptr(&sc910gs_pm_ops),
        .of_match_table = sc910gs_of_match,
    },
    .probe  = sc910gs_probe,
    .remove = sc910gs_remove,
};
module_i2c_driver(sc910gs_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_DESCRIPTION("SmartSense SC910GS sensor driver");
MODULE_LICENSE("GPL");
