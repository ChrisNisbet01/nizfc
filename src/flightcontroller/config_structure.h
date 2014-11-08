#ifndef __CONFIG_STRUCTURE_H__
#define __CONFIG_STRUCTURE_H__

typedef enum config_group_t
{
	config_group_reciever = 0
} config_group_t;

typedef enum poll_id_t
{
	poll_id_run_command,			/* a CLI command to be processed */
	poll_id_save_configuration,		/* saving configuration */
	poll_id_show_configuration		/* show configuration (all, non-default) */
} poll_id_t;

typedef struct config_group_mappings_st
{
	char const * group_name;
	int          (*handler)(poll_id_t poll_id, void *pv, void *user_context);
} config_group_mappings_st;

typedef enum poll_result_t
{
	poll_result_ok,
	poll_result_pending,
	poll_result_error
} poll_result_t;

typedef enum config_data_types_t
{
	config_data_type_boolean,
	config_data_type_int8,
	config_data_type_uint8,
	config_data_type_int16,
	config_data_type_uint16,
	config_data_type_int32,
	config_data_type_uint32,
	config_data_type_float,
	config_data_type_string,
	config_data_type_enum,				/* enumerated type. Accepts integer value or string matching one of the enumerated types. */
	// TODO:
#if 0
	config_data_type_list_int8,			/* a set of int8 values */
	config_data_type_list_uint8,		/* a set of uint8 values */
	config_data_type_list_int16,		/* a set of int16 values */
	config_data_type_list_uint16,		/* a set of uint16 values */
	config_data_type_list_int32,		/* a set of int32 values */
	config_data_type_list_uint32,		/* a set of uint32 values */
	config_data_type_list_float,		/* a set of float values */
	config_data_type_list_string,		/* a set of string values */
	config_data_type_list_enum			/* a set of enumerated types. Accepts integer values or strings matching one of the enumerated types. */
#endif
} config_data_types_t;

typedef struct enum_mapping_st
{
	char const * name;
	int8_t		value;
} enum_mapping_st;

typedef struct enum_data_point_st
{
		enum_mapping_st         const * enum_mappings;			/* pointer to table of enums vs names */
		uint_fast8_t			num_enum_mappings;
} enum_data_point_st;

typedef struct config_data_point_st
{
	char 					const * name;
	config_data_types_t		type;
	uint_fast8_t			offset_to_data_point;
	union
	{
		uint_fast8_t		max_string_length;	/* used when data point type is string */
		enum_data_point_st  enum_data;
	} type_specific;

} config_data_point_st;

/* definitions for extracting fields out of the config header */
#define CONFIG_GROUP_SHIFT				25u
#define CONFIG_GROUP_MASK				0x0000007f
#define CONFIG_INSTANCE_SHIFT			19u
#define CONFIG_INSTANCE_MASK			0x0000003f
#define CONFIG_PARAMETER_ID_SHIFT		12u
#define CONFIG_PARAMETER_ID_MASK		0x0000007f
#define CONFIG_PARAMETER_TYPE_SHIFT		7u
#define CONFIG_PARAMETER_TYPE_MASK		0x0000001f
#define CONFIG_PARAMETER_LENGTH_SHIFT	0
#define CONFIG_PARAMETER_LENGTH_MASK	0x0000007f

#define GET_CONFIG_FIELD( value, field )	((value >> CONFIG_ ## field ## _SHIFT) & CONFIG_ ## field ## _MASK)

typedef struct run_command_data_st
{
	void *pctx;
	int argc;
	char **argv;
} run_command_data_st;

poll_result_t poll_groups( poll_id_t poll_id, void *pv, bool poll_all_groups, void *user_context );

#endif /*  __CONFIG_STRUCTURE_H__ */

