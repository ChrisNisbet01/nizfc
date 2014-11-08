#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <utils.h>
#include <config_structure.h>
#include <cli.h>
#include <receiver.h>

static const char group_name_receiver[] = "rx";

extern int receiver_group_handler( poll_id_t poll_id, void *pv );

static const config_group_mappings_st config_groups[] =
{
	[config_group_reciever] =
		{
			.group_name = group_name_receiver,
			.handler = receiver_group_handler
		}
};

poll_result_t poll_groups( poll_id_t poll_id, void *pv, bool poll_all_groups )
{
	uint_fast32_t index;
	poll_result_t result = poll_result_ok;

	for (index=0; index < ARRAY_SIZE(config_groups); index++)
	{
		result = config_groups[index].handler(poll_id, pv);
		if( poll_all_groups != true && result != poll_result_ok )
			break;
	}

	return result;
}

static char const *enumLookup( int8_t value, enum_mapping_st const * mappings, uint_fast8_t nb_mappings )
{
	uint_fast8_t index;

	for (index=0; index < nb_mappings; index++)
	{
		if (value == mappings[index].value)
			return mappings[index].name;
	}

	/* not found in mappings */
	return NULL;
}

static bool enumLookupByName( char *name, enum_mapping_st const * mappings, uint_fast8_t nb_mappings, uint8_t *enum_value )
{
	uint_fast8_t index;

	for (index=0; index < nb_mappings; index++)
	{
		if (strcasecmp(name, mappings[index].name) == 0)
		{
			*enum_value = mappings[index].value;
			return true;
		}
	}

	/* not found in mappings */
	return false;
}


static void print_configuration_data_point( void *pdata,
											config_data_point_st const * pconfig,
											int (*printfn)( void *pv, const char *fmt, ...),
											void *printfpv )
{
	void *pconfig_data = (char *)pdata + pconfig->offset_to_data_point;

	switch( pconfig->type )
	{
		case config_data_type_boolean:
		{
			int8_t value = *(int8_t *)pconfig_data;

			printfn(printfpv, "%s", (value != 0) ? "ON" : "OFF");
			break;
		}
		case config_data_type_int8:
		{
			int value = *(int8_t *)pconfig_data;

			printfn(printfpv, "%d", value);
			break;
		}
		case config_data_type_int16:
		{
			int value = *(int16_t *)pconfig_data;

			printfn(printfpv, "%d", value);
			break;
		}
		case config_data_type_int32:
		{
			int value = *(int32_t *)pconfig_data;

			printfn(printfpv, "%d", value);
			break;
		}
		case config_data_type_uint8:
		{
			unsigned int value = *(int8_t *)pconfig_data;

			printfn(printfpv, "%u", value);
			break;
		}
		case config_data_type_uint16:
		{
			uint16_t value = *(int16_t *)pconfig_data;

			printfn(printfpv, "%u", value);
			break;
		}
		case config_data_type_uint32:
		{
			uint16_t value = *(int32_t *)pconfig_data;

			printfn(printfpv, "%u", value);
			break;
		}
		case config_data_type_float:
		{
			float value = *(float *)pconfig_data;

			// TODO: printf floating point
			printfn(printfpv, "%f", value);
			break;
		}
		case config_data_type_string:
		{
			char * value = (char *)pconfig_data;

			printfn(printfpv, "%s", value);
			break;
		}
		case config_data_type_enum:
		{
			int8_t value = *(int8_t *)pconfig_data;
			char const * mapping = enumLookup( value,
												pconfig->type_specific.enum_data.enum_mappings,
												pconfig->type_specific.enum_data.num_enum_mappings );

			printfn(printfpv, "%s", (mapping != NULL) ? mapping : "???" );
			break;
		}
	}
}

static const char *true_values[] =
{
	"yes",
	"on",
	"1",
};

bool true_value_lookup( char *value )
{
	uint32_t index;

	for (index=0; index < ARRAY_SIZE(true_values); index++)
	{
		if (strcasecmp( value, true_values[index] ) == 0)
			return true;
	}

	return false;
}

static bool assign_configuration_data_point( void *pdata, config_data_point_st const * pconfig, char *value )
{
	bool wrote_parameter_value = false;
	void *pconfig_data = (char *)pdata + pconfig->offset_to_data_point;
	long val = strtol(value, NULL, 0);

	switch( pconfig->type )
	{
		case config_data_type_boolean:
		{
			int is_true = true_value_lookup( value );

			*(int8_t *)pconfig_data = is_true;
			wrote_parameter_value = true;
			break;
		}
		case config_data_type_int8:
			*(int8_t *)pconfig_data = val;
			wrote_parameter_value = true;
			break;
		case config_data_type_int16:
			*(int16_t *)pconfig_data = val;
			wrote_parameter_value = true;
			break;
		case config_data_type_int32:
			*(int32_t *)pconfig_data = val;
			wrote_parameter_value = true;
			break;
		case config_data_type_uint8:
			*(uint8_t *)pconfig_data = val;
			wrote_parameter_value = true;
			break;
		case config_data_type_uint16:
			*(uint16_t *)pconfig_data = val;
			wrote_parameter_value = true;
			break;
		case config_data_type_uint32:
			*(uint32_t *)pconfig_data = val;
			wrote_parameter_value = true;
			break;
		case config_data_type_float:
			*(float *)pconfig_data = strtof(value, NULL);
			wrote_parameter_value = true;
			break;
		case config_data_type_string:
			strlcpy( pconfig_data, value, pconfig->type_specific.max_string_length );
			wrote_parameter_value = true;
			break;
		case config_data_type_enum:
		{
			uint8_t enum_value;
			bool found_mapping = enumLookupByName( value,
													pconfig->type_specific.enum_data.enum_mappings,
													pconfig->type_specific.enum_data.num_enum_mappings,
													&enum_value );

			if ( found_mapping == false )
			{
				/* see if we can find a matching enum by value */
				if (enumLookup( val,
								pconfig->type_specific.enum_data.enum_mappings,
								pconfig->type_specific.enum_data.num_enum_mappings ) != NULL)
				{
					enum_value = val;
					found_mapping = true;
				}
			}
			if ( found_mapping == true )
			{
				*(uint8_t *)pconfig_data = enum_value;
				wrote_parameter_value = true;
			}
			break;
		}
	}

	return wrote_parameter_value;
}


static config_data_point_st const * config_data_point_lookup( config_data_point_st const * const data_points, uint8_t nb_data_points, char const * parameter_name)
{
	config_data_point_st const *data_point =  data_points;

	while (data_point < data_points + nb_data_points )
	{
		if (strcasecmp( data_point->name, parameter_name ) == 0 )
		{
			return data_point;
		}
		data_point++;
	}

	return NULL;
}

bool print_config_value( void *pdata,
						config_data_point_st const * const data_points,
						uint8_t nb_data_points,
						char const * parameter_name,
						int (*printfn)( void *pv, const char *fmt, ...),
						void *printfpv
						)
{
	config_data_point_st const *data_point;
	bool printed_parameter = false;

	data_point = config_data_point_lookup( data_points, nb_data_points, parameter_name );
	if (data_point != NULL)
	{
		print_configuration_data_point( pdata, data_point, printfn, printfpv );
		printed_parameter = true;
	}

	return printed_parameter;
}

bool assign_config_value( void *pdata, config_data_point_st const * const data_points, uint8_t nb_data_points, char * parameter_name, char *parameter_value)
{
	config_data_point_st const *data_point;
	bool wrote_parameter = false;

	data_point = config_data_point_lookup( data_points, nb_data_points, parameter_name );
	if (data_point != NULL)
	{
		wrote_parameter = assign_configuration_data_point( pdata, data_point, parameter_value );
	}

	return wrote_parameter;
}



