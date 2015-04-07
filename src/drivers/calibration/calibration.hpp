#pragma once

#include <mathlib/mathlib.h>

/*
 * common sensor scaling factors; Vout = Vscale * (Vin - Voffset)
 */

struct __EXPORT calibration_values_s {
	math::Vector<3> offsets;
	math::Vector<3> scales;
	uint32_t date_h;
	float temperature;

	inline calibration_values_s() {
		scales.set(1.0f);
		date_h = 0;
		temperature = -278.15f;
	}
};

/**
 * Inheritance enables overloading for parameter saving
 */
struct __EXPORT accel_calibration_s : calibration_values_s {};
struct __EXPORT gyro_calibration_s : calibration_values_s {};
struct __EXPORT mag_calibration_s : calibration_values_s {};

/*
 * Fills date_h and temperature fields of the structure if available
 * Returns true only if all fields were filled successfully
 */
__EXPORT bool fill_calibration_conditions (calibration_values_s *calibration);

/*
 * Sets parameters specified in offset_params and scale_params as well as date and temperature
 * from values in calibration argument
 */
__EXPORT bool set_calibration_parameters (const char* const offset_params[3], const char* const scale_params[3],
		const char * const date_param, const char * const temp_param, const calibration_values_s &calibration);
/*
 * Helper function to set accelerometer calibration parameters
 */
__EXPORT bool set_calibration_parameters (const accel_calibration_s &accel_calibration);
/*
 * Helper function to set gyroscope calibration parameters
 */
__EXPORT bool set_calibration_parameters (const gyro_calibration_s &gyro_calibration);
/*
 * Helper function to set magnetometer calibration parameters
 */
__EXPORT bool set_calibration_parameters (const mag_calibration_s &mag_calibration);

/*
 * Gets parameters specified in offset_params and scale_params as well as date and temperature
 * and stores them in calibration argument
 * Failure may result in incomplete parameter structure
 */
__EXPORT bool get_calibration_parameters (const char* const offset_params[3], const char* const scale_params[3],
		const char * const date_param, const char * const temp_param, calibration_values_s *calibration);
/*
 * Helper function to get accelerometer calibration parameters
 * Failure may result in incomplete parameter structure
 */
__EXPORT bool get_calibration_parameters (accel_calibration_s *accel_calibration);
/*
 * Helper function to get gyroscope calibration parameters
 * Failure may result in incomplete parameter structure
 */
__EXPORT bool get_calibration_parameters (gyro_calibration_s *gyro_calibration);
/*
 * Helper function to get magnetometer calibration parameters
 * Failure may result in incomplete parameter structure
 */
__EXPORT bool get_calibration_parameters (mag_calibration_s *mag_calibration);

/*
 * Print calibration values from calibration argument to console
 * If mavlink_fd argument is nonzero, then sends mavlink_info message too
 */
__EXPORT void print_calibration(const calibration_values_s &calibration, int mavlink_fd = 0);
