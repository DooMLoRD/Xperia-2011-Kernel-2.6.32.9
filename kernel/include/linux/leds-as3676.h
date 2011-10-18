#ifndef __LINUX_AS3676_H
#define __LINUX_AS3676_H

/*    ^
 *    |          k2 /------- y3
 *    |           \/
 *    |        x2 /
 *    |         |/
 *    |      k1 /
 *    |       \/
 *    |       /
 * y0-|------|
 *    |      x1
 *    +------------------------>
 */

enum as3676_amb_value {
	AS3676_AMB_OFF,
	AS3676_AMB_GROUP_1,
	AS3676_AMB_GROUP_2,
	AS3676_AMB_GROUP_3,
	AS3676_AMB_MAX,

	AS3676_AMB_MASK = 0x03
};

struct as3676_als_curve {
	int y0; /* Current multiplier for < x1 */
	int y3; /* Current multiplier for high values */
	int k1; /* k1/32 = slope for > x1 < x2 */
	int k2; /* k2/32 = slope for > x2 */
	int x1;
	int x2;
};

enum as3676_filter {
	AS3676_FILTER_0_25HZ	= 0,
	AS3676_FILTER_0_5HZ	= 1,
	AS3676_FILTER_1HZ	= 2,
	AS3676_FILTER_2HZ	= 3,
	AS3676_FILTER_4HZ	= 4,
	AS3676_FILTER_8HZ	= 5,
	AS3676_FILTER_16HZ	= 6,
	AS3676_FILTER_32HZ	= 7,

	AS3676_FILTER_MAX,
};

enum as3676_gain {
	AS3676_GAIN_0_25	= 0,
	AS3676_GAIN_0_5		= 1,
	AS3676_GAIN_1		= 2,
	AS3676_GAIN_2		= 3,

	AS3676_GAIN_MAX,
};

enum as3676_als_source {
	AS3676_ALS_SOURCE_VANA	= 1,
	AS3676_ALS_SOURCE_AUDIO	= 3,
	AS3676_ALS_SOURCE_GPIO1	= 2,
	AS3676_ALS_SOURCE_GPIO2	= 0,
	AS3676_ALS_SOURCE_DCDC	= 22,
};

struct as3676_als_config {
	enum as3676_filter filter_up;
	enum as3676_filter filter_down;
	enum as3676_gain gain;
	int offset;
	enum as3676_als_source source;
	struct as3676_als_curve curve[AS3676_AMB_MAX];
};

struct as3676_audio_config {
	int current_3x;
	int audio_control;
	int audio_input;
	int audio_output;
};

enum as3676_sinks {
	AS3676_SINK_01,
	AS3676_SINK_02,
	AS3676_SINK_06,
	AS3676_SINK_30,
	AS3676_SINK_31,
	AS3676_SINK_32,
	AS3676_SINK_33,
	AS3676_SINK_41,
	AS3676_SINK_42,
	AS3676_SINK_43,
	AS3676_SINK_RGB1,
	AS3676_SINK_RGB2,
	AS3676_SINK_RGB3,

	AS3676_SINK_MAX,
};

enum as3676_led_flags {
	AS3676_FLAG_RGB		= (1 << 0), /* is RGB */
	AS3676_FLAG_BLINK	= (1 << 1), /* should have blinking */
	AS3676_FLAG_ALS_GROUP1	= (1 << 2), /* connected to the ALS group1 */
	AS3676_FLAG_ALS_GROUP2	= (1 << 3), /* connected to the ALS group2 */
	AS3676_FLAG_ALS_GROUP3	= (1 << 4), /* connected to the ALS group3 */
	AS3676_FLAG_PWM_INIT	= (1 << 5), /* should turn on slowly once */
	AS3676_FLAG_PWM_CTRL	= (1 << 6),
	AS3676_FLAG_DLS	= (1 << 7), /* should be connected to the DLS */
	AS3676_FLAG_AUDIO 	= (1 << 8), /* should be connected to audio*/
	AS3676_FLAG_WAIT_RESUME	= (1 << 9),
					/* should not control during suspend */
	AS3676_FLAG_ALS	= AS3676_FLAG_ALS_GROUP1,
	AS3676_FLAG_ALS_MASK	= (AS3676_FLAG_ALS_GROUP1 |
					AS3676_FLAG_ALS_GROUP2 |
					AS3676_FLAG_ALS_GROUP3),
};

struct as3676_platform_led {
	const char *name;
	int sinks;
	int flags;
	int max_current;
	int default_brightness;
};

enum as3676_ldo {
	AS3676_LDO_MIN = 1800,
	AS3676_LDO_MAX = 3350,
};

struct as3676_platform_data {
	struct as3676_platform_led *leds;
	int als_connected;
	int als_wait;
	int dls_connected;
	int num_leds;
	struct as3676_als_config *als_config;
	struct as3676_audio_config *audio_config;
	int ldo_mV;
};

#endif
