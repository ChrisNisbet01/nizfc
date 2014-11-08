#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <config_structure.h>
#include <cli.h>
#include <receiver.h>

extern int receiver_poll_handler( poll_id_t poll_id, void *pv );

static const code_group_mappings_st config_groups[] =
{
	{ .poll_handler = receiver_poll_handler }
};

poll_result_t poll_groups( poll_id_t poll_id, void *pv, bool poll_all_groups )
{
	uint_fast32_t index;
	poll_result_t result = poll_result_ok;

	for (index=0; index < ARRAY_SIZE(config_groups); index++)
	{
		result = config_groups[index].poll_handler(poll_id, pv);
		if( poll_all_groups != true && result != poll_result_ok )
			break;
	}

	return result;
}


