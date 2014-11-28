#ifndef __CONFIG_STRUCTURE_H__
#define __CONFIG_STRUCTURE_H__

/* do not change these values as they're used to identify saved config parameters */
typedef enum configuration_id_t
{
	configuration_id_reserved = 0,	/* used as padding at the end of the config. */
	configuration_id_receiver = 1,
	configuration_id_save = 2,
	configuration_id_show = 3,
	configuration_id_output = 4,
	configuration_id_roll = 5,
	configuration_id_pitch = 6,
	configuration_id_yaw = 7,
	configuration_id_failsafe = 8,
	configuration_id_board = 9,
	configuration_id_aux = 10,
	configuration_id_acc = 11,
	configuration_id_gyro = 12,
	configuration_id_mag = 13,
	configuration_id_angle = 14,
	configuration_id_rate = 15
} configuration_id_t;

/*
	These values are used to identify the data types stored in FLASH.
	It is important that they retain their values over time.
*/
typedef enum config_data_types_t
{
	config_data_type_boolean = 0,
	config_data_type_int8 = 1,
	config_data_type_uint8 = 2,
	config_data_type_int16 = 3,
	config_data_type_uint16 = 4,
	config_data_type_int32 = 5,
	config_data_type_uint32 = 6,
	config_data_type_float = 7,
	config_data_type_string = 8,
	config_data_type_enum = 9,				/* enumerated type. Accepts integer value or string matching one of the enumerated types. */
} config_data_types_t;

typedef struct enum_mapping_st
{
	char const * name;
	int8_t		value;
} enum_mapping_st;

typedef struct enumDataType_st
{
		enum_mapping_st         const * mappings;			/* pointer to table of enums vs names */
		uint_fast8_t			num_mappings;
} enumDataType_st;

typedef void (* customParameterSetValueFn)( unsigned int parameter_id, void * value );

typedef struct parameterConfig_st
{
	unsigned int			parameter_id;		/* configuration specific ID. Used to obtain name and when storing to FLASH */
	config_data_types_t		data_type;
	uint_fast8_t			offsetToData;
	union
	{
		uint_fast8_t		max_string_length;	/* used when data_type is string */
		enumDataType_st     enum_data;
	} type_specific;

} parameterConfig_st;

/* definitions for extracting fields out of the config header */
#define CONFIG_GROUP_MASK				0x0fe00000	/* 7 bits */
#define CONFIG_GROUP_SHIFT				21u

#define CONFIG_INSTANCE_MASK			0x001fc000	/* 7 bits */
#define CONFIG_INSTANCE_SHIFT			14u

#define CONFIG_PARAMETER_ID_MASK		0x00003f80	/* 7 bits */
#define CONFIG_PARAMETER_ID_SHIFT		7u

#define CONFIG_PARAMETER_TYPE_MASK		0x0000007f	/* 7 bits */
#define CONFIG_PARAMETER_TYPE_SHIFT		0u

#define GET_CONFIG_FIELD( value, field )	(((uint32_t)(value) & CONFIG_ ## field ## _MASK) >> CONFIG_ ## field ## _SHIFT)
#define MAKE_CONFIG_FIELD_VALUE( value, field )	((value << CONFIG_ ## field ## _SHIFT) & CONFIG_ ## field ## _MASK)


#endif /*  __CONFIG_STRUCTURE_H__ */

