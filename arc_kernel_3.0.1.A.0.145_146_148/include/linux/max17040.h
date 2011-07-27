#ifndef __MAX17040_H_
#define __MAX17040_H_

#define MAX17040_NAME		"max17040"
#define MAX17040_REG_SIZE       2
#define MAX17040_MODEL_NBR_BLKS 4
#define MAX17040_MODEL_BLK_SIZE 16

struct max17040_model_desc {
	u8 ocv_test[MAX17040_REG_SIZE];
	u8 soc_low;
	u8 soc_high;
	u8 model_data[MAX17040_MODEL_NBR_BLKS][MAX17040_MODEL_BLK_SIZE];
	int exp;
};

struct max17040_rcomp_data {
	u8 rcomp0;
	int temp_co_hot;
	int temp_co_cold;
	int temp_div;
};

struct max17040_platform_data {
	struct max17040_model_desc model_desc;
	struct max17040_rcomp_data rcomp_data;
};

#endif
