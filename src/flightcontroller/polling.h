#ifndef __POLLING_H__
#define __POLLING_H__

typedef enum poll_id_t
{
	poll_id_identify,				/* identify all config commands */
	poll_id_identify_non_config,	/* identify all other commands */
	poll_id_initialise,				/* load default configurations etc */
	poll_id_load_configuration,		/* load saved parameter values */
	poll_id_run_command,			/* a CLI command to be processed */
	poll_id_run_non_config_command,	/* a CLI command to be processed */
	poll_id_save_configuration,		/* called from 'save' command. saving configuration */
	poll_id_show_configuration		/* called from 'show' command. show configuration (all, non-default) */
} poll_id_t;

typedef struct code_group_mappings_st
{
	int          	(*pollHandler)(poll_id_t poll_id, void *pv);
} code_group_mappings_st;

typedef enum poll_result_t
{
	poll_result_ok,
	poll_result_pending,
	poll_result_error
} poll_result_t;

poll_result_t pollCodeGroups( poll_id_t poll_id, void *pv, bool poll_all_groups );

#endif /*  __POLLING_H__ */
