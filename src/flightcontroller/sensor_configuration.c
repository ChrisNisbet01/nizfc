#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f30x_tim.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>
#include <coos.h>
#include <utils.h>
#include <polling.h>
#include <cmds.h>
#include <configuration_commands.h>
#include <cli.h>
#include <sensor_configuration.h>


static int sensor_command( run_command_data_st *pcommand );
static char const * sensorParameterNameLookup( unsigned int parameterID );

sensor_configuration_st acc_configuration[NB_ACC_CONFIGURATIONS];
sensor_configuration_st gyro_configuration[NB_GYRO_CONFIGURATIONS];
sensor_configuration_st mag_configuration[NB_MAG_CONFIGURATIONS];

static const sensor_configuration_st default_sensor_configuration =
{
	.lpf_factor = 0.0f	/* no filtering applied */
};

typedef enum sensor_parameter_id_t
{
	parameter_id_lpf = 0,
} sensor_parameter_id_t;

static char const * const parameter_name_mappings[] =
{
	[parameter_id_lpf] = "lpf",
};

static const parameterConfig_st sensor_config_parameterConfigs[] =
{
	{
	.parameter_id = parameter_id_lpf,
	.data_type = config_data_type_uint16,
	.offsetToData = offsetof(sensor_configuration_st, lpf_factor),
	}
};

static const command_st sensor_commands[] =
{
	{
		.group_id = configuration_id_acc,
		.name = "acc",
		.handler = sensor_command,
		.configuration = acc_configuration,
		.nb_configuration_instance = ARRAY_SIZE(acc_configuration),
		.configuration_size = sizeof(acc_configuration[0]),
		.default_configuration = &default_sensor_configuration,
		.parameterConfigs = sensor_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(sensor_config_parameterConfigs),
		.ParameterNameLookupCB = sensorParameterNameLookup
	},
	{
		.group_id = configuration_id_gyro,
		.name = "gyro",
		.handler = sensor_command,
		.configuration = gyro_configuration,
		.nb_configuration_instance = ARRAY_SIZE(gyro_configuration),
		.configuration_size = sizeof(gyro_configuration[0]),
		.default_configuration = &default_sensor_configuration,
		.parameterConfigs = sensor_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(sensor_config_parameterConfigs),
		.ParameterNameLookupCB = sensorParameterNameLookup
	},
	{
		.group_id = configuration_id_mag,
		.name = "mag",
		.handler = sensor_command,
		.configuration = mag_configuration,
		.nb_configuration_instance = ARRAY_SIZE(mag_configuration),
		.configuration_size = sizeof(mag_configuration[0]),
		.default_configuration = &default_sensor_configuration,
		.parameterConfigs = sensor_config_parameterConfigs,
		.nbParameterConfigs = ARRAY_SIZE(sensor_config_parameterConfigs),
		.ParameterNameLookupCB = sensorParameterNameLookup
	},
};

static char const * sensorParameterNameLookup( unsigned int parameterID )
{
	if (parameterID < ARRAY_SIZE(parameter_name_mappings) )
		return parameter_name_mappings[parameterID];

	/* shouldn't happen */
	return "";
}

static int sensor_command( run_command_data_st *pcommand )
{
	return handleStandardCommand( pcommand );
}

static void initSensorConfiguration( void )
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(acc_configuration); index++ )
		memcpy( &acc_configuration[index], &default_sensor_configuration, sizeof acc_configuration[index] );
	for (index = 0; index < ARRAY_SIZE(gyro_configuration); index++ )
		memcpy( &gyro_configuration[index], &default_sensor_configuration, sizeof gyro_configuration[index] );
	for (index = 0; index < ARRAY_SIZE(mag_configuration); index++ )
		memcpy( &mag_configuration[index], &default_sensor_configuration, sizeof mag_configuration[index] );
}

int sensorPollHandler( poll_id_t poll_id, void *pv )
{
	int result = poll_result_error;

	switch( poll_id )
	{
		case poll_id_identify:
			result = idCommandHandler( sensor_commands, ARRAY_SIZE(sensor_commands), pv );
			break;
		case poll_id_initialise:
			initSensorConfiguration();
			result = poll_result_ok;
			break;
		case poll_id_run_command:
			result = runCommandHandler( sensor_commands, ARRAY_SIZE(sensor_commands), pv );
			break;
		// TODO: handle new config event after parameter written
		// TODO: apply current config into running config
		case poll_id_save_configuration:
		{
			(void)saveParameterValues( pv,
								sensor_commands,
								ARRAY_SIZE(sensor_commands)
								);
			result = poll_result_ok;
			break;
		}
		case poll_id_show_configuration:
			(void)printParametersHandler( pv,
								sensor_commands,
								ARRAY_SIZE(sensor_commands),
								parameter_name_mappings
								);
			result = poll_result_ok;
			break;
		case poll_id_load_configuration:
			(void)loadParametersHandler( pv,
								sensor_commands,
								ARRAY_SIZE(sensor_commands)
								);
			result = poll_result_ok;
			break;
		default:
			break;
	}

	return result;
}


