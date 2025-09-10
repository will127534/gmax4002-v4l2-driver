// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony GMAX4002 camera.
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
#define GMAX4002_STREAM_DELAY_US          25000
#define GMAX4002_STREAM_DELAY_RANGE_US    1000

/* Initialisation delay between XCLR low->high and the moment sensor is ready */
#define GMAX4002_XCLR_MIN_DELAY_US        10000
#define GMAX4002_XCLR_DELAY_RANGE_US      1000


/* Exposure control (lines) */
/* The driver is set to use external exposure, it will still report the control but it actuall does nothing. */
#define GMAX4002_EXPOSURE_MIN             1000
#define GMAX4002_EXPOSURE_STEP            1000
#define GMAX4002_EXPOSURE_DEFAULT         1000
#define GMAX4002_EXPOSURE_MAX             1000

/* Black level control */
#define GMAX4002_REG_BLKLEVEL             CCI_REG16_LE(0x305B)
#define GMAX4002_BLKLEVEL_DEFAULT         50

/* Analog gain control */
#define GMAX4002_REG_ANALOG_GAIN          CCI_REG8(0x2EC9)
#define GMAX4002_ANA_GAIN_MIN             0
#define GMAX4002_ANA_GAIN_MAX             15
#define GMAX4002_ANA_GAIN_STEP            1
#define GMAX4002_ANA_GAIN_DEFAULT         0

/* Vertical Flip */
#define GMAX4002_REG_FLIP_H            CCI_REG8(0x3002)
#define GMAX4002_REG_FLIP_V            CCI_REG8(0x2E05)

/* Pixel rate helper (sensor line clock proxy used below) */
#define GMAX4002_PIXEL_RATE               60000000U
#define GMAX4002_LINK_FREQ               600000000U

static const s64 gmax4002_link_freq_menu[] = {
    GMAX4002_LINK_FREQ,
};


/* gmax4002 native and active pixel array size. 
#define GMAX4002_NATIVE_WIDTH       2080U
#define GMAX4002_NATIVE_HEIGHT      1216U
#define GMAX4002_PIXEL_ARRAY_LEFT     16U
#define GMAX4002_PIXEL_ARRAY_TOP       4U
*/

/* Technically the native width and height is 2080x1216 but the frame output is 2048Ux1218 */
#define GMAX4002_NATIVE_WIDTH       2048U
#define GMAX4002_NATIVE_HEIGHT      1218U
#define GMAX4002_PIXEL_ARRAY_LEFT      0U
#define GMAX4002_PIXEL_ARRAY_TOP      18U
#define GMAX4002_PIXEL_ARRAY_WIDTH  2048U
#define GMAX4002_PIXEL_ARRAY_HEIGHT 1200U


static const struct cci_reg_sequence mode_common_regs[] = {
	{CCI_REG8(0x2E00),0x00},
	{CCI_REG8(0x2E01),0x01},
	{CCI_REG8(0x2E02),0x00},
	{CCI_REG8(0x2E03),0x06},
	{CCI_REG8(0x2E04),0x00},
	{CCI_REG8(0x2E05),0x08},
	{CCI_REG8(0x2E06),0x02},
	{CCI_REG8(0x2E07),0x00},
	{CCI_REG8(0x2E08),0x00},
	{CCI_REG8(0x2E09),0x00},
	{CCI_REG8(0x2E0A),0x00},
	{CCI_REG8(0x2E0B),0x01},
	{CCI_REG8(0x2E0C),0x00},
	{CCI_REG8(0x2E0D),0xBE},
	{CCI_REG8(0x2E0E),0x04},
	{CCI_REG8(0x2E0F),0x01},
	{CCI_REG8(0x2E10),0x00},
	{CCI_REG8(0x2E11),0x12},
	{CCI_REG8(0x2E12),0x00},
	{CCI_REG8(0x2E13),0x00},
	{CCI_REG8(0x2E14),0xBA},
	{CCI_REG8(0x2E15),0x04},
	{CCI_REG8(0x2E16),0x00},
	{CCI_REG8(0x2E17),0x00},
	{CCI_REG8(0x2E18),0x08},
	{CCI_REG8(0x2E19),0x00},
	{CCI_REG8(0x2E1A),0xB0},
	{CCI_REG8(0x2E1B),0x04},
	{CCI_REG8(0x2E1C),0x00},
	{CCI_REG8(0x2E1D),0x00},
	{CCI_REG8(0x2E1E),0x00},
	{CCI_REG8(0x2E1F),0x00},
	{CCI_REG8(0x2E20),0x00},
	{CCI_REG8(0x2E21),0x00},
	{CCI_REG8(0x2E22),0x00},
	{CCI_REG8(0x2E23),0x00},
	{CCI_REG8(0x2E24),0x00},
	{CCI_REG8(0x2E25),0x00},
	{CCI_REG8(0x2E26),0x00},
	{CCI_REG8(0x2E27),0x00},
	{CCI_REG8(0x2E28),0x00},
	{CCI_REG8(0x2E29),0x00},
	{CCI_REG8(0x2E2A),0x00},
	{CCI_REG8(0x2E2B),0x00},
	{CCI_REG8(0x2E2C),0x00},
	{CCI_REG8(0x2E2D),0x00},
	{CCI_REG8(0x2E2E),0x00},
	{CCI_REG8(0x2E2F),0x00},
	{CCI_REG8(0x2E30),0x00},
	{CCI_REG8(0x2E31),0x00},
	{CCI_REG8(0x2E32),0x00},
	{CCI_REG8(0x2E33),0x00},
	{CCI_REG8(0x2E34),0x00},
	{CCI_REG8(0x2E35),0x00},
	{CCI_REG8(0x2E36),0x00},
	{CCI_REG8(0x2E37),0x00},
	{CCI_REG8(0x2E38),0x00},
	{CCI_REG8(0x2E39),0x00},
	{CCI_REG8(0x2E3A),0x00},
	{CCI_REG8(0x2E3B),0x00},
	{CCI_REG8(0x2E3C),0x00},
	{CCI_REG8(0x2E3D),0x00},
	{CCI_REG8(0x2E3E),0x00},
	{CCI_REG8(0x2E3F),0x00},
	{CCI_REG8(0x2E40),0x00},
	{CCI_REG8(0x2E41),0x00},
	{CCI_REG8(0x2E42),0x00},
	{CCI_REG8(0x2E43),0x00},
	{CCI_REG8(0x2E44),0x00},
	{CCI_REG8(0x2E45),0x00},
	{CCI_REG8(0x2E46),0x00},
	{CCI_REG8(0x2E47),0x00},
	{CCI_REG8(0x2E48),0x00},
	{CCI_REG8(0x2E49),0x00},
	{CCI_REG8(0x2E4A),0x00},
	{CCI_REG8(0x2E4B),0x00},
	{CCI_REG8(0x2E4C),0x00},
	{CCI_REG8(0x2E4D),0x00},
	{CCI_REG8(0x2E4E),0x00},
	{CCI_REG8(0x2E4F),0x00},
	{CCI_REG8(0x2E50),0x00},
	{CCI_REG8(0x2E51),0x00},
	{CCI_REG8(0x2E52),0x00},
	{CCI_REG8(0x2E53),0x00},
	{CCI_REG8(0x2E54),0x00},
	{CCI_REG8(0x2E55),0x00},
	{CCI_REG8(0x2E56),0x00},
	{CCI_REG8(0x2E57),0x00},
	{CCI_REG8(0x2E58),0x01},
	{CCI_REG8(0x2E59),0x14},
	{CCI_REG8(0x2E5A),0x00},
	{CCI_REG8(0x2E5B),0x00},
	{CCI_REG8(0x2E5C),0x04},
	{CCI_REG8(0x2E5D),0x22},
	{CCI_REG8(0x2E5E),0x01},
	{CCI_REG8(0x2E5F),0x02},
	{CCI_REG8(0x2E60),0x02},
	{CCI_REG8(0x2E61),0x02},
	{CCI_REG8(0x2E62),0x06},
	{CCI_REG8(0x2E63),0x28},
	{CCI_REG8(0x2E64),0x01},
	{CCI_REG8(0x2E65),0x00},
	{CCI_REG8(0x2E66),0x02},
	{CCI_REG8(0x2E67),0x00},
	{CCI_REG8(0x2E68),0x24},
	{CCI_REG8(0x2E69),0x05},
	{CCI_REG8(0x2E6A),0x62},
	{CCI_REG8(0x2E6B),0x03},
	{CCI_REG8(0x2E6C),0x46},
	{CCI_REG8(0x2E6D),0x78},
	{CCI_REG8(0x2E6E),0x02},
	{CCI_REG8(0x2E6F),0x2A},
	{CCI_REG8(0x2E70),0xFF},
	{CCI_REG8(0x2E71),0xFF},
	{CCI_REG8(0x2E72),0xFF},
	{CCI_REG8(0x2E73),0xFF},
	{CCI_REG8(0x2E74),0x32},
	{CCI_REG8(0x2E75),0x64},
	{CCI_REG8(0x2E76),0x14},
	{CCI_REG8(0x2E77),0xFF},
	{CCI_REG8(0x2E78),0xFF},
	{CCI_REG8(0x2E79),0xFF},
	{CCI_REG8(0x2E7A),0x01},
	{CCI_REG8(0x2E7B),0x87},
	{CCI_REG8(0x2E7C),0xFF},
	{CCI_REG8(0x2E7D),0xFF},
	{CCI_REG8(0x2E7E),0x01},
	{CCI_REG8(0x2E7F),0x88},
	{CCI_REG8(0x2E80),0x05},
	{CCI_REG8(0x2E81),0x24},
	{CCI_REG8(0x2E82),0xFF},
	{CCI_REG8(0x2E83),0xFF},
	{CCI_REG8(0x2E84),0x05},
	{CCI_REG8(0x2E85),0x31},
	{CCI_REG8(0x2E86),0x5C},
	{CCI_REG8(0x2E87),0x84},
	{CCI_REG8(0x2E88),0x03},
	{CCI_REG8(0x2E89),0x0C},
	{CCI_REG8(0x2E8A),0x13},
	{CCI_REG8(0x2E8B),0x34},
	{CCI_REG8(0x2E8C),0xFF},
	{CCI_REG8(0x2E8D),0xFF},
	{CCI_REG8(0x2E8E),0xFF},
	{CCI_REG8(0x2E8F),0xFF},
	{CCI_REG8(0x2E90),0xFF},
	{CCI_REG8(0x2E91),0xFF},
	{CCI_REG8(0x2E92),0xFF},
	{CCI_REG8(0x2E93),0xFF},
	{CCI_REG8(0x2E94),0x04},
	{CCI_REG8(0x2E95),0x0C},
	{CCI_REG8(0x2E96),0x14},
	{CCI_REG8(0x2E97),0x34},
	{CCI_REG8(0x2E98),0x01},
	{CCI_REG8(0x2E99),0x02},
	{CCI_REG8(0x2E9A),0x11},
	{CCI_REG8(0x2E9B),0x12},
	{CCI_REG8(0x2E9C),0x04},
	{CCI_REG8(0x2E9D),0x0C},
	{CCI_REG8(0x2E9E),0x14},
	{CCI_REG8(0x2E9F),0x34},
	{CCI_REG8(0x2EA0),0x0B},
	{CCI_REG8(0x2EA1),0x0C},
	{CCI_REG8(0x2EA2),0x33},
	{CCI_REG8(0x2EA3),0x34},
	{CCI_REG8(0x2EA4),0x0C},
	{CCI_REG8(0x2EA5),0x14},
	{CCI_REG8(0x2EA6),0x10},
	{CCI_REG8(0x2EA7),0xFF},
	{CCI_REG8(0x2EA8),0x01},
	{CCI_REG8(0x2EA9),0x02},
	{CCI_REG8(0x2EAA),0x11},
	{CCI_REG8(0x2EAB),0x12},
	{CCI_REG8(0x2EAC),0x01},
	{CCI_REG8(0x2EAD),0x02},
	{CCI_REG8(0x2EAE),0xFF},
	{CCI_REG8(0x2EAF),0xFF},
	{CCI_REG8(0x2EB0),0xFF},
	{CCI_REG8(0x2EB1),0xFF},
	{CCI_REG8(0x2EB2),0xFF},
	{CCI_REG8(0x2EB3),0xFF},
	{CCI_REG8(0x2EB4),0xFF},
	{CCI_REG8(0x2EB5),0xFF},
	{CCI_REG8(0x2EB6),0x2E},
	{CCI_REG8(0x2EB7),0x1C},
	{CCI_REG8(0x2EB8),0x1E},
	{CCI_REG8(0x2EB9),0x0C},
	{CCI_REG8(0x2EBA),0x03},
	{CCI_REG8(0x2EBB),0x00},
	{CCI_REG8(0x2EBC),0x01},
	{CCI_REG8(0x2EBD),0x00},
	{CCI_REG8(0x2EBE),0x01},
	{CCI_REG8(0x2EBF),0x03},
	{CCI_REG8(0x2EC0),0x01},
	{CCI_REG8(0x2EC1),0x01},
	{CCI_REG8(0x2EC2),0x00},
	{CCI_REG8(0x2EC3),0x01},
	{CCI_REG8(0x2EC4),0x1E},
	{CCI_REG8(0x2EC5),0x0C},
	{CCI_REG8(0x2EC6),0x00},
	{CCI_REG8(0x2EC7),0x00},
	{CCI_REG8(0x2EC8),0x01},
	{CCI_REG8(0x2EC9),0x01},
	{CCI_REG8(0x2ECA),0x03},
	{CCI_REG8(0x2ECB),0x01},
	{CCI_REG8(0x3000),0x01},
	{CCI_REG8(0x3001),0x02},
	{CCI_REG8(0x3002),0x00},
	{CCI_REG8(0x3003),0x00},
	{CCI_REG8(0x3004),0x03},
	{CCI_REG8(0x3005),0x00},
	{CCI_REG8(0x3006),0x08},
	{CCI_REG8(0x3007),0x10},
	{CCI_REG8(0x3008),0x00},
	{CCI_REG8(0x3009),0x04},
	{CCI_REG8(0x300A),0x01},
	{CCI_REG8(0x300B),0x00},
	{CCI_REG8(0x300C),0x01},
	{CCI_REG8(0x300D),0x0F},
	{CCI_REG8(0x300E),0x00},
	{CCI_REG8(0x300F),0x01},
	{CCI_REG8(0x3010),0x01},
	{CCI_REG8(0x3011),0x01},
	{CCI_REG8(0x3012),0x00},
	{CCI_REG8(0x3013),0x00},
	{CCI_REG8(0x3014),0x8E},
	{CCI_REG8(0x3015),0x09},
	{CCI_REG8(0x3016),0x04},
	{CCI_REG8(0x3017),0x00},
	{CCI_REG8(0x3018),0x08},
	{CCI_REG8(0x3019),0x07},
	{CCI_REG8(0x301A),0x10},
	{CCI_REG8(0x301B),0x07},
	{CCI_REG8(0x301C),0x27},
	{CCI_REG8(0x301D),0x00},
	{CCI_REG8(0x301E),0x0B},
	{CCI_REG8(0x301F),0x09},
	{CCI_REG8(0x3020),0x05},
	{CCI_REG8(0x3021),0x06},
	{CCI_REG8(0x3022),0x96},
	{CCI_REG8(0x3023),0xF8},
	{CCI_REG8(0x3024),0x14},
	{CCI_REG8(0x3025),0x00},
	{CCI_REG8(0x3026),0x00},
	{CCI_REG8(0x3027),0x00},
	{CCI_REG8(0x3028),0x00},
	{CCI_REG8(0x3029),0x00},
	{CCI_REG8(0x302A),0x04},
	{CCI_REG8(0x302B),0x04},
	{CCI_REG8(0x302C),0x04},
	{CCI_REG8(0x302D),0x04},
	{CCI_REG8(0x302E),0x00},
	{CCI_REG8(0x302F),0x00},
	{CCI_REG8(0x3030),0x00},
	{CCI_REG8(0x3031),0x00},
	{CCI_REG8(0x3039),0x00},
	{CCI_REG8(0x303A),0x00},
	{CCI_REG8(0x303B),0x00},
	{CCI_REG8(0x303C),0x00},
	{CCI_REG8(0x303D),0x01},
	{CCI_REG8(0x303E),0x00},
	{CCI_REG8(0x303F),0x00},
	{CCI_REG8(0x3040),0x10},
	{CCI_REG8(0x3041),0x00},
	{CCI_REG8(0x3042),0x10},
	{CCI_REG8(0x3043),0x00},
	{CCI_REG8(0x3044),0x19},
	{CCI_REG8(0x3045),0x10},
	{CCI_REG8(0x3046),0x00},
	{CCI_REG8(0x3047),0x00},
	{CCI_REG8(0x3048),0x00},
	{CCI_REG8(0x3049),0x00},
	{CCI_REG8(0x304A),0x00},
	{CCI_REG8(0x304B),0x00},
	{CCI_REG8(0x304C),0x00},
	{CCI_REG8(0x304D),0x00},
	{CCI_REG8(0x304E),0x00},
	{CCI_REG8(0x304F),0x02},
	{CCI_REG8(0x3050),0x0A},
	{CCI_REG8(0x3051),0x02},
	{CCI_REG8(0x3052),0x00},
	{CCI_REG8(0x3053),0x10},
	{CCI_REG8(0x3054),0x00},
	{CCI_REG8(0x3055),0x00},
	{CCI_REG8(0x3056),0x5E},
	{CCI_REG8(0x3057),0x01},
	{CCI_REG8(0x3058),0x00},
	{CCI_REG8(0x3059),0x01},
	{CCI_REG8(0x305A),0x00},
	{CCI_REG8(0x305B),0x10},
	{CCI_REG8(0x305C),0x00},
	{CCI_REG8(0x305D),0x04},
	{CCI_REG8(0x305E),0x00},
	{CCI_REG8(0x305F),0x00},
	{CCI_REG8(0x3060),0x00},
	{CCI_REG8(0x3200),0x20},
	{CCI_REG8(0x3201),0x00},
	{CCI_REG8(0x3202),0x04},
	{CCI_REG8(0x3203),0x03},
	{CCI_REG8(0x3204),0x20},
	{CCI_REG8(0x3205),0x03},
	{CCI_REG8(0x3206),0x03},
	{CCI_REG8(0x3207),0x03},
	{CCI_REG8(0x3208),0x16},
	{CCI_REG8(0x3209),0x04},
	{CCI_REG8(0x320A),0x00},
	{CCI_REG8(0x320B),0x16},
	{CCI_REG8(0x320C),0x04},
	{CCI_REG8(0x320D),0x1A},
	{CCI_REG8(0x320E),0x0F},
	{CCI_REG8(0x320F),0x00},
	{CCI_REG8(0x3210),0x11},
	{CCI_REG8(0x3211),0x07},
	{CCI_REG8(0x3212),0x00},
	{CCI_REG8(0x3213),0x0E},
	{CCI_REG8(0x3214),0x1B},
	{CCI_REG8(0x3215),0x03},
	{CCI_REG8(0x3216),0x3F},
	{CCI_REG8(0x3217),0x04},
	{CCI_REG8(0x3218),0x07},
	{CCI_REG8(0x3219),0x00},
	{CCI_REG8(0x321A),0x3F},
	{CCI_REG8(0x321B),0x07},
	{CCI_REG8(0x321C),0x00},
	{CCI_REG8(0x321D),0x04},
	{CCI_REG8(0x321E),0x3F},
	{CCI_REG8(0x321F),0x04},
	{CCI_REG8(0x3220),0x07},
	{CCI_REG8(0x3221),0x00},
	{CCI_REG8(0x3222),0x14},
	{CCI_REG8(0x3223),0x03},
	{CCI_REG8(0x3224),0x00},
	{CCI_REG8(0x3225),0x00},
	{CCI_REG8(0x3226),0x3F},
	{CCI_REG8(0x3227),0x03},
	{CCI_REG8(0x3228),0x00},
	{CCI_REG8(0x3229),0x00},
	{CCI_REG8(0x322A),0x06},
	{CCI_REG8(0x322B),0x03},
	{CCI_REG8(0x322C),0x1B},
	{CCI_REG8(0x322D),0x00},
	{CCI_REG8(0x322E),0x07},
	{CCI_REG8(0x322F),0x03},
	{CCI_REG8(0x3230),0x0E},
	{CCI_REG8(0x3231),0x41},
	{CCI_REG8(0x3232),0x53},
	{CCI_REG8(0x3233),0x4E},
	{CCI_REG8(0x3234),0x47},
	{CCI_REG8(0x3235),0x47},
	{CCI_REG8(0x3236),0x47},
	{CCI_REG8(0x3237),0x47},
	{CCI_REG8(0x3238),0x50},
	{CCI_REG8(0x3239),0x47},
	{CCI_REG8(0x323A),0x53},
	{CCI_REG8(0x323B),0x53},
	{CCI_REG8(0x323C),0x4A},
	{CCI_REG8(0x323D),0x4A},
	{CCI_REG8(0x323E),0x68},
	{CCI_REG8(0x323F),0x0A},
	{CCI_REG8(0x3240),0x00},
	{CCI_REG8(0x3241),0x1A},
	{CCI_REG8(0x3242),0x20},
	{CCI_REG8(0x3243),0x04},
	{CCI_REG8(0x3244),0x13},
	{CCI_REG8(0x3245),0x13},
	{CCI_REG8(0x3246),0x30},
	{CCI_REG8(0x3247),0x30},
	{CCI_REG8(0x3248),0x7D},
	{CCI_REG8(0x3249),0x1C},
	{CCI_REG8(0x324A),0x1E},
	{CCI_REG8(0x324B),0x00},
	{CCI_REG8(0x324C),0x00},
	{CCI_REG8(0x324D),0x00},
	{CCI_REG8(0x324E),0x00},
	{CCI_REG8(0x3300),0x00},
	{CCI_REG8(0x3301),0x00},
	{CCI_REG8(0x3302),0x00},
	{CCI_REG8(0x3303),0x00},
	{CCI_REG8(0x3304),0x00},
	{CCI_REG8(0x3305),0x00},
	{CCI_REG8(0x3306),0x00},
	{CCI_REG8(0x3307),0x01},
	{CCI_REG8(0x3308),0x00},
	{CCI_REG8(0x3311),0xD0},
	{CCI_REG8(0x3312),0x07},
	{CCI_REG8(0x3313),0xB8},
	{CCI_REG8(0x3314),0x0B},
	{CCI_REG8(0x3315),0xF4},
	{CCI_REG8(0x3316),0x01},
	{CCI_REG8(0x3317),0xE8},
	{CCI_REG8(0x3318),0x03},
	{CCI_REG8(0x3319),0xE8},
	{CCI_REG8(0x331A),0x03},
	{CCI_REG8(0x331B),0xE8},
	{CCI_REG8(0x331C),0x03},
	{CCI_REG8(0x331D),0xA0},
	{CCI_REG8(0x331E),0x0F},
	{CCI_REG8(0x331F),0xD0},
	{CCI_REG8(0x3320),0x07},
	{CCI_REG8(0x3321),0x01},
	{CCI_REG8(0x3322),0x00},
	{CCI_REG8(0x3323),0x01},
	{CCI_REG8(0x3324),0x00},
	{CCI_REG8(0x3325),0x01},
	{CCI_REG8(0x3326),0x00},
	{CCI_REG8(0x3327),0x01},
	{CCI_REG8(0x3328),0x00},
	{CCI_REG8(0x3329),0x01},
	{CCI_REG8(0x332A),0x00},
	{CCI_REG8(0x332B),0x01},
	{CCI_REG8(0x332C),0x00},
	{CCI_REG8(0x332D),0xA0},
	{CCI_REG8(0x332E),0x0F},
	{CCI_REG8(0x332F),0xE8},
	{CCI_REG8(0x3330),0x03},
	{CCI_REG8(0x3331),0xD0},
	{CCI_REG8(0x3332),0x07},
	{CCI_REG8(0x3333),0x03},
	{CCI_REG8(0x3334),0x00},
	{CCI_REG8(0x3335),0xA0},
	{CCI_REG8(0x3336),0x0F},
	{CCI_REG8(0x3337),0xD0},
	{CCI_REG8(0x3338),0x07},
	{CCI_REG8(0x3339),0xF4},
	{CCI_REG8(0x333A),0x01},
	{CCI_REG8(0x333B),0x3C},
	{CCI_REG8(0x333C),0x00},
	{CCI_REG8(0x333D),0xB8},
	{CCI_REG8(0x333E),0x0B},
	{CCI_REG8(0x333F),0xE8},
	{CCI_REG8(0x3340),0x03},
	{CCI_REG8(0x3341),0xE8},
	{CCI_REG8(0x3342),0x03},
	{CCI_REG8(0x3343),0xE8},
	{CCI_REG8(0x3344),0x03},
	{CCI_REG8(0x3345),0x55},
	{CCI_REG8(0x3346),0x55},
	{CCI_REG8(0x3347),0x55},
	{CCI_REG8(0x3348),0x55},
	{CCI_REG8(0x3349),0x55},
	{CCI_REG8(0x334A),0x55},
	{CCI_REG8(0x334B),0x55},
	{CCI_REG8(0x334C),0x55},
	{CCI_REG8(0x334D),0x10},
	{CCI_REG8(0x334E),0x32},
	{CCI_REG8(0x334F),0x54},
	{CCI_REG8(0x3350),0x9A},
	{CCI_REG8(0x3351),0xCD},
	{CCI_REG8(0x3352),0x7B},
	{CCI_REG8(0x3353),0xE8},
	{CCI_REG8(0x3354),0x6F},
	{CCI_REG8(0x3355),0x10},
	{CCI_REG8(0x3356),0x42},
	{CCI_REG8(0x3357),0x95},
	{CCI_REG8(0x3358),0xEA},
	{CCI_REG8(0x3359),0xCD},
	{CCI_REG8(0x335A),0x3B},
	{CCI_REG8(0x335B),0x87},
	{CCI_REG8(0x335C),0x6E},
	{CCI_REG8(0x335D),0x00},
	{CCI_REG8(0x335E),0x00},
	{CCI_REG8(0x335F),0x00},
	{CCI_REG8(0x3360),0x00},
	{CCI_REG8(0x3361),0x01},
	{CCI_REG8(0x3362),0x00},
	{CCI_REG8(0x3400),0x00},
	{CCI_REG8(0x3401),0x12},
	{CCI_REG8(0x3402),0x00},
	{CCI_REG8(0x3403),0x00},
	{CCI_REG8(0x3404),0x64},
	{CCI_REG8(0x3405),0x02},
};


/* Mode description */
struct gmax4002_mode {
    unsigned int width;
    unsigned int height;
    struct v4l2_rect crop;
    struct {
        unsigned int num_of_regs;
        const struct cci_reg_sequence *regs;
    } reg_list;
};

static struct gmax4002_mode supported_modes_10bit[] = {
    {
        .width = GMAX4002_NATIVE_WIDTH,
        .height = GMAX4002_NATIVE_HEIGHT,
        .crop = {
            .left = GMAX4002_PIXEL_ARRAY_LEFT,
            .top = GMAX4002_PIXEL_ARRAY_TOP,
            .width = GMAX4002_PIXEL_ARRAY_WIDTH,
            .height = GMAX4002_PIXEL_ARRAY_HEIGHT,
        },
    },
};

/* Formats exposed per mode/bit depth */
static const u32 codes[] = {
    /* 10-bit modes. */
    MEDIA_BUS_FMT_SRGGB10_1X10,
    MEDIA_BUS_FMT_SGRBG10_1X10,
    MEDIA_BUS_FMT_SGBRG10_1X10,
    MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const u32 mono_codes[] = {
	MEDIA_BUS_FMT_Y10_1X10,
};

/* Regulators */
static const char * const gmax4002_supply_name[] = {
    "vana", /* 3.3V analog */
    "vdig", /* 1.1V core   */
    "vddl", /* 1.8V I/O    */
};

#define GMAX4002_NUM_SUPPLIES ARRAY_SIZE(gmax4002_supply_name)

/* --------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------
 */

struct gmax4002 {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct device *dev;
    struct regmap *regmap;

    struct clk *xclk;

    struct gpio_desc *reset_gpio;
    struct regulator_bulk_data supplies[GMAX4002_NUM_SUPPLIES];

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

    bool streaming;
};

/* Helpers */

static inline struct gmax4002 *to_gmax4002(struct v4l2_subdev *sd)
{
    return container_of(sd, struct gmax4002, sd);
}

static inline void get_mode_table(struct gmax4002 *gmax4002, unsigned int code,
                  const struct gmax4002_mode **mode_list,
                  unsigned int *num_modes)
{
    switch (code) {
    case MEDIA_BUS_FMT_SRGGB10_1X10:
    case MEDIA_BUS_FMT_SGRBG10_1X10:
    case MEDIA_BUS_FMT_SGBRG10_1X10:
    case MEDIA_BUS_FMT_SBGGR10_1X10:
        *mode_list = supported_modes_10bit;
        *num_modes = ARRAY_SIZE(supported_modes_10bit);
        break;
    default:
        *mode_list = NULL;
        *num_modes = 0;
    }
}

static u32 gmax4002_get_format_code(struct gmax4002 *gmax4002, u32 code)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(codes); i++)
        if (codes[i] == code)
            return codes[i];
    return codes[0];
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

static int gmax4002_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct gmax4002 *gmax4002 = container_of(ctrl->handler, struct gmax4002, ctrl_handler);
    const struct gmax4002_mode *mode, *mode_list;
    struct v4l2_subdev_state *state;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int num_modes;
    int ret = 0;

    /* Apply control only when powered (runtime active). */
    if (!pm_runtime_get_if_active(gmax4002->dev))
        return 0;

    /* Look, I told you this is a bare minimum driver with external exposure contorl
     * The only control that works is analog gain, VFLIP, HFLIP.
     */
    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE: {
        //dev_info(gmax4002->dev, "EXPOSURE=%u -> SHR=%u (VMAX=%u HMAX=%u)\n", ctrl->val, 0, 0, 0);
    }
    case V4L2_CID_ANALOGUE_GAIN:
        dev_info(gmax4002->dev, "ANALOG_GAIN=%u\n", ctrl->val);
        ret = cci_write(gmax4002->regmap, GMAX4002_REG_ANALOG_GAIN, ctrl->val, NULL);
        if (ret)
            dev_err_ratelimited(gmax4002->dev, "Gain write failed (%d)\n", ret);
        break;
    case V4L2_CID_VBLANK: {
        //dev_info(gmax4002->dev, "VBLANK=%u -> VMAX=%u\n", ctrl->val, 0);
        break;
    }
    case V4L2_CID_HBLANK: {
        //dev_info(gmax4002->dev, "HBLANK=%u -> HMAX=%u\n", ctrl->val, 0);
        break;
    }
    case V4L2_CID_VFLIP:
        ret = cci_write(gmax4002->regmap, GMAX4002_REG_FLIP_V, ctrl->val, NULL);
        break;
    case V4L2_CID_HFLIP:
        ret = cci_write(gmax4002->regmap, GMAX4002_REG_FLIP_H, ctrl->val, NULL);
        break;
    case V4L2_CID_BRIGHTNESS: {
        //u16 blacklevel = min_t(u32, ctrl->val, 1023);
        //ret = cci_write(gmax4002->regmap, GMAX4002_REG_BLKLEVEL, blacklevel, NULL);
        break;
    }
    default:
        dev_info(gmax4002->dev, "Unhandled ctrl %s: id=0x%x, val=0x%x\n",
            ctrl->name, ctrl->id, ctrl->val);
        break;
    }

    pm_runtime_put(gmax4002->dev);
    return ret;
}

static const struct v4l2_ctrl_ops gmax4002_ctrl_ops = {
    .s_ctrl = gmax4002_set_ctrl,
};


static int gmax4002_init_controls(struct gmax4002 *gmax4002)
{
    struct v4l2_ctrl_handler *hdl = &gmax4002->ctrl_handler;
    struct v4l2_fwnode_device_properties props;
    int ret;

    ret = v4l2_ctrl_handler_init(hdl, 16);

    /* Read-only, updated per mode */
    gmax4002->pixel_rate = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops,
                           V4L2_CID_PIXEL_RATE,
                           1, GMAX4002_PIXEL_RATE, 1, 1);
    gmax4002->link_freq =
        v4l2_ctrl_new_int_menu(hdl, &gmax4002_ctrl_ops, V4L2_CID_LINK_FREQ,
                       0, 0, gmax4002_link_freq_menu);
    if (gmax4002->link_freq)
        gmax4002->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    gmax4002->vblank = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops,
                       V4L2_CID_VBLANK, 0, 0xFFFFF, 1, 0);
    gmax4002->hblank = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops,
                       V4L2_CID_HBLANK, 0, 0xFFFF, 1, 0);

    gmax4002->exposure = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops,
                         V4L2_CID_EXPOSURE,
                         GMAX4002_EXPOSURE_MIN, GMAX4002_EXPOSURE_MAX,
                         GMAX4002_EXPOSURE_STEP, GMAX4002_EXPOSURE_DEFAULT);

    gmax4002->gain = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                     GMAX4002_ANA_GAIN_MIN, GMAX4002_ANA_GAIN_MAX,
                     GMAX4002_ANA_GAIN_STEP, GMAX4002_ANA_GAIN_DEFAULT);

    gmax4002->vflip = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops,
                      V4L2_CID_VFLIP, 0, 1, 1, 0);

    gmax4002->hflip = v4l2_ctrl_new_std(hdl, &gmax4002_ctrl_ops,
                      V4L2_CID_HFLIP, 0, 1, 1, 0);
    if (hdl->error) {
        ret = hdl->error;
        dev_err(gmax4002->dev, "control init failed (%d)\n", ret);
        goto err_free;
    }

    ret = v4l2_fwnode_device_parse(gmax4002->dev, &props);
    if (ret)
        goto err_free;

    ret = v4l2_ctrl_new_fwnode_properties(hdl, &gmax4002_ctrl_ops, &props);
    if (ret)
        goto err_free;

    gmax4002->sd.ctrl_handler = hdl;
    return 0;

err_free:
    v4l2_ctrl_handler_free(hdl);
    return ret;
}

static void gmax4002_free_controls(struct gmax4002 *gmax4002)
{
    v4l2_ctrl_handler_free(gmax4002->sd.ctrl_handler);
}

/* --------------------------------------------------------------------------
 * Pad ops / formats
 * --------------------------------------------------------------------------
 */

static int gmax4002_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    unsigned int entries;
    const u32 *tbl;

    tbl = codes;
    entries = ARRAY_SIZE(codes) / 4;

    if (code->index >= entries)
        return -EINVAL;

    code->code = gmax4002_get_format_code(gmax4002, tbl[code->index * 4]);
    return 0;
}

static int gmax4002_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_frame_size_enum *fse)
{
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    const struct gmax4002_mode *mode_list;
    unsigned int num_modes;

    get_mode_table(gmax4002, fse->code, &mode_list, &num_modes);
    if (fse->index >= num_modes)
        return -EINVAL;
    if (fse->code != gmax4002_get_format_code(gmax4002, fse->code))
        return -EINVAL;

    fse->min_width  = mode_list[fse->index].width;
    fse->max_width  = fse->min_width;
    fse->min_height = mode_list[fse->index].height;
    fse->max_height = fse->min_height;

    return 0;
}

static int gmax4002_set_pad_format(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_format *fmt)
{
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    const struct gmax4002_mode *mode_list, *mode;
    unsigned int num_modes;
    struct v4l2_mbus_framefmt *format;
    struct v4l2_rect *crop;

    /* Normalize requested code to what we really support */
    fmt->format.code = gmax4002_get_format_code(gmax4002, fmt->format.code);

    get_mode_table(gmax4002, fmt->format.code, &mode_list, &num_modes);
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

static int gmax4002_enable_streams(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state, u32 pad,
                 u64 streams_mask)
{
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    const struct gmax4002_mode *mode_list, *mode;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;
    int ret;

    ret = pm_runtime_get_sync(gmax4002->dev);
    if (ret < 0) {
        pm_runtime_put_noidle(gmax4002->dev);
        return ret;
    }

    ret = cci_multi_reg_write(gmax4002->regmap, mode_common_regs,
                  ARRAY_SIZE(mode_common_regs), NULL);
    if (ret) {
        dev_err(gmax4002->dev, "Failed to write common settings\n");
        goto err_rpm_put;
    }

    usleep_range(10000,12000);
    //CLK_STABLE_EN = 1
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x2E00), BIT(0), BIT(0), &ret);
    //PWR_UP_EN = 1
    cci_write(gmax4002->regmap, CCI_REG8(0x3301), 0x01, &ret);

    // > 80ms
    usleep_range(100000,101000);
    /* D<0x3024>_Bit<5>=0 */
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x3024), BIT(5), 0, &ret);
    /* D<0x3023>_Bit<7:4>=b'1111 */
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x3023), GENMASK(7, 4),FIELD_PREP(GENMASK(7, 4), 0xF), &ret);
    /* D<0x3023>_Bit<0>=1 */
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x3023), BIT(0), BIT(0), &ret);
    /* D<0x3023>_Bit<2:1>=2'b11 */
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x3023), GENMASK(2, 1),FIELD_PREP(GENMASK(2, 1), 0x3), &ret);
    /* D<0x3024>_Bit<5>=1 */
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x3024), BIT(5), BIT(5), &ret);
    /* D<0x3023>_Bit<2>=0 */
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x3023), BIT(2), 0, &ret);

    //STREAM_EN = 1
    cci_update_bits(gmax4002->regmap, CCI_REG8(0x2E00), BIT(1), BIT(1), &ret);


    /* Apply user controls after writing the base tables */
    ret = __v4l2_ctrl_handler_setup(gmax4002->sd.ctrl_handler);
    if (ret) {
        dev_err(gmax4002->dev, "Control handler setup failed\n");
        goto err_rpm_put;
    }

    dev_info(gmax4002->dev, "Streaming started\n");
    usleep_range(GMAX4002_STREAM_DELAY_US,
             GMAX4002_STREAM_DELAY_US + GMAX4002_STREAM_DELAY_RANGE_US);

    /* vflip cannot change during streaming */
    __v4l2_ctrl_grab(gmax4002->vflip, true);
    __v4l2_ctrl_grab(gmax4002->hflip, true);

    return 0;

err_rpm_put:
    pm_runtime_put_autosuspend(gmax4002->dev);
    return ret;
}

static int gmax4002_disable_streams(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *state, u32 pad,
                  u64 streams_mask)
{
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    int ret;

    __v4l2_ctrl_grab(gmax4002->vflip, false);
    __v4l2_ctrl_grab(gmax4002->hflip, false);

    pm_runtime_put_autosuspend(gmax4002->dev);

    return ret;
}

/* --------------------------------------------------------------------------
 * Power / runtime PM
 * --------------------------------------------------------------------------
 */

static int gmax4002_power_on(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    int ret;

    dev_info(gmax4002->dev, "power_on\n");

    ret = regulator_bulk_enable(GMAX4002_NUM_SUPPLIES, gmax4002->supplies);
    if (ret) {
        dev_err(gmax4002->dev, "Failed to enable regulators\n");
        return ret;
    }

    ret = clk_prepare_enable(gmax4002->xclk);
    if (ret) {
        dev_err(gmax4002->dev, "Failed to enable clock\n");
        goto reg_off;
    }

    gpiod_set_value_cansleep(gmax4002->reset_gpio, 1);
    usleep_range(GMAX4002_XCLR_MIN_DELAY_US,
             GMAX4002_XCLR_MIN_DELAY_US + GMAX4002_XCLR_DELAY_RANGE_US);
    return 0;

reg_off:
    regulator_bulk_disable(GMAX4002_NUM_SUPPLIES, gmax4002->supplies);
    return ret;
}

static int gmax4002_power_off(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct gmax4002 *gmax4002 = to_gmax4002(sd);

    dev_info(gmax4002->dev, "power_off\n");

    gpiod_set_value_cansleep(gmax4002->reset_gpio, 0);
    regulator_bulk_disable(GMAX4002_NUM_SUPPLIES, gmax4002->supplies);
    clk_disable_unprepare(gmax4002->xclk);

    return 0;
}

/* --------------------------------------------------------------------------
 * Selection / state
 * --------------------------------------------------------------------------
 */

static int gmax4002_get_selection(struct v4l2_subdev *sd,
                struct v4l2_subdev_state *sd_state,
                struct v4l2_subdev_selection *sel)
{
    struct gmax4002 *gmax4002 = to_gmax4002(sd);
    const struct gmax4002_mode *mode_list, *mode;
    const struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;

    fmt = v4l2_subdev_state_get_format(sd_state, 0);
    get_mode_table(gmax4002, fmt->code, &mode_list, &n_modes);
    mode = v4l2_find_nearest_size(mode_list, n_modes, width, height,
                                  fmt->width, fmt->height);

    switch (sel->target) {


    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.left   = 0;
        sel->r.top    = 0;
        sel->r.width  = GMAX4002_NATIVE_WIDTH;   /* pixel array (no blanking) */
        sel->r.height = GMAX4002_NATIVE_HEIGHT;
        return 0;
    case V4L2_SEL_TGT_CROP_BOUNDS:
    case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = GMAX4002_PIXEL_ARRAY_TOP;
		sel->r.left = GMAX4002_PIXEL_ARRAY_LEFT;
		sel->r.width = GMAX4002_PIXEL_ARRAY_WIDTH;
		sel->r.height = GMAX4002_PIXEL_ARRAY_HEIGHT;
        return 0;

    case V4L2_SEL_TGT_CROP:
        sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
        return 0;

    default:
        return -EINVAL;
    }
}

static int gmax4002_init_state(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state)
{
    struct v4l2_rect *crop;
    struct v4l2_subdev_format fmt = {
        .which  = V4L2_SUBDEV_FORMAT_TRY,
        .pad    = 0,
        .format = {
            .code   = MEDIA_BUS_FMT_SRGGB10_1X10,
            .width  = GMAX4002_NATIVE_WIDTH,
            .height = GMAX4002_NATIVE_HEIGHT,
        },
    };

    gmax4002_set_pad_format(sd, state, &fmt);

    crop = v4l2_subdev_state_get_crop(state, 0);
    *crop = supported_modes_10bit[0].crop;

    return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */

static const struct v4l2_subdev_video_ops gmax4002_video_ops = {
    .s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gmax4002_pad_ops = {
    .enum_mbus_code = gmax4002_enum_mbus_code,
    .get_fmt        = v4l2_subdev_get_fmt,
    .set_fmt        = gmax4002_set_pad_format,
    .get_selection  = gmax4002_get_selection,
    .enum_frame_size = gmax4002_enum_frame_size,
    .enable_streams  = gmax4002_enable_streams,
    .disable_streams = gmax4002_disable_streams,
};

static const struct v4l2_subdev_internal_ops gmax4002_internal_ops = {
    .init_state = gmax4002_init_state,
};

static const struct v4l2_subdev_ops gmax4002_subdev_ops = {
    .video = &gmax4002_video_ops,
    .pad   = &gmax4002_pad_ops,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 * --------------------------------------------------------------------------
 */

static int gmax4002_check_hwcfg(struct device *dev, struct gmax4002 *gmax4002)
{
    struct fwnode_handle *endpoint;
    struct v4l2_fwnode_endpoint ep = {
        .bus_type = V4L2_MBUS_CSI2_DPHY,
    };
    int ret = -EINVAL;

    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "endpoint node not found\n");
        return -EINVAL;
    }

    if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep)) {
        dev_err(dev, "could not parse endpoint\n");
        goto out_put;
    }

    if (ep.bus.mipi_csi2.num_data_lanes != 4) {
        dev_err(dev, "only 4 data lanes supported\n");
        goto out_free;
    }

    ret = 0;

out_free:
    v4l2_fwnode_endpoint_free(&ep);
out_put:
    fwnode_handle_put(endpoint);
    return ret;
}

static int gmax4002_get_regulators(struct gmax4002 *gmax4002)
{
    unsigned int i;

    for (i = 0; i < GMAX4002_NUM_SUPPLIES; i++)
        gmax4002->supplies[i].supply = gmax4002_supply_name[i];

    return devm_regulator_bulk_get(gmax4002->dev,
                       GMAX4002_NUM_SUPPLIES, gmax4002->supplies);
}

static int gmax4002_check_module_exists(struct gmax4002 *gmax4002)
{
    int ret;
    u64 val;

    /* No chip-id register; read a known register as a presence test */
    ret = cci_read(gmax4002->regmap, GMAX4002_REG_BLKLEVEL, &val, NULL);
    if (ret) {
        dev_err(gmax4002->dev, "register read failed (%d)\n", ret);
        return ret;
    }

    dev_info(gmax4002->dev, "Sensor detected\n");
    return 0;
}

static int gmax4002_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct gmax4002 *gmax4002;
    unsigned int xclk_freq;
    int ret, i;
    const char *sync_mode;

    gmax4002 = devm_kzalloc(dev, sizeof(*gmax4002), GFP_KERNEL);
    if (!gmax4002)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&gmax4002->sd, client, &gmax4002_subdev_ops);
    gmax4002->dev = dev;

    ret = gmax4002_check_hwcfg(dev, gmax4002);
    if (ret)
        return ret;

    gmax4002->regmap = devm_cci_regmap_init_i2c(client, 16);
    if (IS_ERR(gmax4002->regmap))
        return dev_err_probe(dev, PTR_ERR(gmax4002->regmap), "CCI init failed\n");

    gmax4002->xclk = devm_clk_get(dev, NULL);
    if (IS_ERR(gmax4002->xclk))
        return dev_err_probe(dev, PTR_ERR(gmax4002->xclk), "xclk missing\n");

    xclk_freq = clk_get_rate(gmax4002->xclk);

    ret = gmax4002_get_regulators(gmax4002);
    if (ret)
        return dev_err_probe(dev, ret, "regulators\n");

    gmax4002->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

    /* Power on to probe the device */
    ret = gmax4002_power_on(dev);
    if (ret)
        return ret;

    ret = gmax4002_check_module_exists(gmax4002);
    if (ret)
        goto err_power_off;

    pm_runtime_set_active(dev);
    pm_runtime_get_noresume(dev);
    pm_runtime_enable(dev);
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);

    ret = gmax4002_init_controls(gmax4002);
    if (ret)
        goto err_pm;

    gmax4002->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    gmax4002->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    gmax4002->sd.internal_ops = &gmax4002_internal_ops;

    gmax4002->pad.flags = MEDIA_PAD_FL_SOURCE;

    ret = media_entity_pads_init(&gmax4002->sd.entity, 1, &gmax4002->pad);
    if (ret) {
        dev_err(dev, "entity pads init failed: %d\n", ret);
        goto err_ctrls;
    }

    gmax4002->sd.state_lock = gmax4002->ctrl_handler.lock;
    ret = v4l2_subdev_init_finalize(&gmax4002->sd);
    if (ret) {
        dev_err_probe(dev, ret, "subdev init\n");
        goto err_entity;
    }

    ret = v4l2_async_register_subdev_sensor(&gmax4002->sd);
    if (ret) {
        dev_err(dev, "sensor subdev register failed: %d\n", ret);
        goto err_entity;
    }

    pm_runtime_mark_last_busy(dev);
    pm_runtime_put_autosuspend(dev);
    return 0;

err_entity:
    media_entity_cleanup(&gmax4002->sd.entity);
err_ctrls:
    gmax4002_free_controls(gmax4002);
err_pm:
    pm_runtime_disable(dev);
    pm_runtime_set_suspended(dev);
err_power_off:
    gmax4002_power_off(dev);
    return ret;
}

static void gmax4002_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct gmax4002 *gmax4002 = to_gmax4002(sd);

    v4l2_async_unregister_subdev(sd);
    v4l2_subdev_cleanup(sd);
    media_entity_cleanup(&sd->entity);
    gmax4002_free_controls(gmax4002);

    pm_runtime_disable(gmax4002->dev);
    if (!pm_runtime_status_suspended(gmax4002->dev))
        gmax4002_power_off(gmax4002->dev);
    pm_runtime_set_suspended(gmax4002->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(gmax4002_pm_ops, gmax4002_power_off,
                 gmax4002_power_on, NULL);

static const struct of_device_id gmax4002_of_match[] = {
    { .compatible = "gpixel,gmax4002" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gmax4002_of_match);

static struct i2c_driver gmax4002_i2c_driver = {
    .driver = {
        .name  = "gmax4002",
        .pm    = pm_ptr(&gmax4002_pm_ops),
        .of_match_table = gmax4002_of_match,
    },
    .probe  = gmax4002_probe,
    .remove = gmax4002_remove,
};
module_i2c_driver(gmax4002_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_DESCRIPTION("Gpixel GMAX4002 sensor driver");
MODULE_LICENSE("GPL");
