#ifndef __SAVE_CONFIGURATION_H__
#define __SAVE_CONFIGURATION_H__

int save_configuration( run_command_data_st const * command_context,
					configuration_id_t configuration_id,
					void const * pcfg,
					unsigned int const nb_configurations,
					unsigned int const configuration_size,
					void const * default_configuration,
					config_data_point_st const * data_points,
					unsigned int const nb_data_points
					);

#endif /* __SAVE_CONFIGURATION_H__ */
