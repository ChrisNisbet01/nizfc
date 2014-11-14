#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <polling.h>

extern int outputPollHandler( poll_id_t poll_id, void *pv );
extern int receiverPollHandler( poll_id_t poll_id, void *pv );
extern int configPollHandler( poll_id_t poll_id, void *pv );
extern int rollPitchPollHandler( poll_id_t poll_id, void *pv );

static const code_group_mappings_st code_groups[] =
{
	{ .pollHandler = outputPollHandler },
	{ .pollHandler = receiverPollHandler },
	{ .pollHandler = configPollHandler },
	{ .pollHandler = rollPitchPollHandler }
};

poll_result_t pollCodeGroups( poll_id_t poll_id, void *pv, bool poll_all_groups )
{
	uint_fast32_t index;
	poll_result_t result = poll_result_error;

	for (index=0; index < ARRAY_SIZE(code_groups); index++)
	{
		result = code_groups[index].pollHandler(poll_id, pv);
		if( poll_all_groups != true && result == poll_result_ok )
			break;
	}

	return result;
}


