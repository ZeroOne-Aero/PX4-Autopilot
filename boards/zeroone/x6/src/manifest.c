/****************************************************************************
 * X6 hardware manifest
 ****************************************************************************/

#include <nuttx/config.h>
#include <board_config.h>
#include <px4_platform_common/board_common.h>

static const px4_hw_mft_item_t x6_manifest[] = {
	{
		.id = PX4_MFT_PX4IO,
		.present = 1,
		.mandatory = 1,
		.connection = px4_hw_con_onboard,
	},
	{
		.id = PX4_MFT_USB,
		.present = 1,
		.mandatory = 1,
		.connection = px4_hw_con_onboard,
	},
	{
		.id = PX4_MFT_CAN2,
		.present = 1,
		.mandatory = 0,
		.connection = px4_hw_con_onboard,
	},
	{
		.id = PX4_MFT_CAN3,
		.present = 0,
		.mandatory = 0,
		.connection = px4_hw_con_unknown,
	},
	{
		.id = PX4_MFT_PM2,
		.present = 0,
		.mandatory = 0,
		.connection = px4_hw_con_unknown,
	},
	{
		.id = PX4_MFT_ETHERNET,
		.present = 1,
		.mandatory = 0,
		.connection = px4_hw_con_onboard,
	},
};

__EXPORT px4_hw_mft_item board_query_manifest(px4_hw_mft_item_id_t id)
{
	if (id < (px4_hw_mft_item_id_t)(sizeof(x6_manifest) / sizeof(x6_manifest[0]))) {
		return &x6_manifest[id];
	}

	static const px4_hw_mft_item_t unsupported = {
		.present = 0,
		.mandatory = 0,
		.connection = px4_hw_con_unknown,
	};

	return &unsupported;
}
