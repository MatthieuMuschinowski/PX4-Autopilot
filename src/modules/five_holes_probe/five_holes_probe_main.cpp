/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file five_holes_probe_main.cpp
 * Five holes probe module
 *
 * @author Matthieu Muschinowski	<matthieu.muschinowski@gmail.com>
 */

#include "five_holes_probe_main.hpp"

//using namespace matrix;

// FixedwingRateControl::FixedwingRateControl(bool vtol) :
// 	ModuleParams(nullptr),
// 	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
// 	_actuator_controls_status_pub(vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0)),
// 	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_fw) : ORB_ID(vehicle_torque_setpoint)),
// 	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_fw) : ORB_ID(vehicle_thrust_setpoint)),
// 	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))

FiveHolesProbe::FiveHolesProbe() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_wind_measure_pub(ORB_ID(wind_measure)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
}

FiveHolesProbe::~FiveHolesProbe()
{
	perf_free(_loop_perf);
}

bool
FiveHolesProbe::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("five holes probes callback registration failed!");
		return false;
	}
	_debug_cnt = 0;

	return true;
}

void
FiveHolesProbe::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void
FiveHolesProbe::Run()
{
	// if (should_exit()) {
	// 	_vehicle_status_sub.unregisterCallback();
	// 	exit_and_cleanup();
	// 	return;
	// }

	perf_begin(_loop_perf);

	parameter_update_poll();

	_wind_measure.timestamp_sample = hrt_absolute_time();

	// First check which probes are actually connected & updated			_solano.windspeed_f = _solano_2.windspeed_f;

	if (_solano_subs[0].update(&_solano_0)){
		if(fabsf(_solano_0.windspeed) > 0.0f ){
			_probe0 = true;
			_solano = _solano_0;
		}
		else _probe0 = false;
	}
	if (_solano_subs[1].update(&_solano_1)){
		if(fabsf(_solano_1.windspeed) > 0.0f ){
			_probe1 = true;
			_solano = _solano_1;
		}
		else _probe1 = false;
	}
	if (_solano_subs[2].update(&_solano_2)){
		if(fabsf(_solano_2.windspeed) > 0.0f ){
			_probe2 = true;
			_solano = _solano_2;
		}
		else _probe2 = false;
	}

	/* run controller on vehicle attitude changes */
	vehicle_attitude_s vehicle_attitude;

	/* check for updates in vehicle attitude topic */
	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {

		//ADD Multiprobe support : for now just check which probe is main
		if((_probe0 && _probe1) || (_probe0 && _probe2) || (_probe1 && _probe2)){
			if(_probe0){
				if(_solano_0.id == 1){
					_solano = _solano_0;
				}
				else if (_solano_0.id == 2){
					_solano_sec = _solano_0;
				}
				else if (_solano_0.id == 3){
					_solano_third = _solano_0;
				}
			}
			if(_probe1){
				if(_solano_1.id == 1){
					_solano = _solano_1;
				}
				else if (_solano_1.id == 2){
					_solano_sec = _solano_1;
				}
				else if (_solano_1.id == 3){
					_solano_third = _solano_1;
				}
			}
			if(_probe2){
				if(_solano_2.id == 1){
					_solano = _solano_2;
				}
				else if (_solano_2.id == 2){
					_solano_sec = _solano_2;
				}
				else if (_solano_2.id == 3){
					_solano_third = _solano_2;
				}
			}
		}

		const matrix::Eulerf euler(matrix::Quatf(vehicle_attitude.q));

		_vehicle_roll = euler.phi();
		_vehicle_pitch = euler.theta();
		_vehicle_yaw = euler.psi();

		_probe_roll = _vehicle_roll +	_param_sens_sol_roll.get()*(float)M_PI / 180.0f;
		_probe_pitch = _vehicle_pitch +	_param_sens_sol_pitch.get()*(float)M_PI / 180.0f;
		_probe_yaw = _vehicle_yaw +	_param_sens_sol_yaw.get()*(float)M_PI / 180.0f;
		// _roll_deg = _roll * 180.0f / (float)M_PI;
		// _pitch_deg = _pitch * 180.0f / (float)M_PI;
		// _yaw_deg = _yaw * 180.0f / (float)M_PI;


		if (_vehicle_air_data_sub.updated()) {
			_vehicle_air_data_sub.update(&_vehicle_air_data);
		}

		if (_vehicle_status_sub.updated()) {
			_vehicle_status_sub.update(&_vehicle_status);
		}

		if(_vehicle_local_position_sub.updated()){
			_vehicle_local_position_sub.update(&_vehicle_local_position);
		}


		float cosEW = cosf(_solano.angle_ew*(float)M_PI / 180.0f);
		float cosNS = cosf(_solano.angle_ns*(float)M_PI / 180.0f);
		float sinEW = sinf(_solano.angle_ew*(float)M_PI / 180.0f);
		float sinNS = sinf(_solano.angle_ns*(float)M_PI / 180.0f);

		float wind_x = _solano.windspeed;
		float wind_y = _solano.windspeed * sinEW / cosEW;
		float wind_z = _solano.windspeed * sinNS / cosNS;


		float cosEW_f = cosf(_solano.angle_ew_f*(float)M_PI / 180.0f);
		float cosNS_f = cosf(_solano.angle_ns_f*(float)M_PI / 180.0f);
		float sinEW_f = sinf(_solano.angle_ew_f*(float)M_PI / 180.0f);
		float sinNS_f = sinf(_solano.angle_ns_f*(float)M_PI / 180.0f);

		float wind_x_f = _solano.windspeed_f;
		float wind_y_f = _solano.windspeed_f * sinEW_f / cosEW_f;
		float wind_z_f = _solano.windspeed_f * sinNS_f / cosNS_f;


		float cosRoll = cosf(_probe_roll);
		float cosPitch = cosf(_probe_pitch);
		float cosYaw = cosf(_probe_yaw);
		float sinRoll = sinf(_probe_roll);
		float sinPitch = sinf(_probe_pitch);
		float sinYaw = sinf(_probe_yaw);

		_wind_measure.windspeed_north = wind_x * cosYaw*cosPitch + wind_y * (cosYaw*sinPitch*sinRoll - sinYaw*cosPitch) + wind_z * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
		_wind_measure.windspeed_east = wind_x * sinYaw*cosPitch + wind_y * (sinYaw*sinPitch*sinRoll + cosYaw*cosPitch) + wind_z * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);
		_wind_measure.windspeed_down = -wind_x * sinPitch + wind_y * cosPitch*sinRoll + wind_z * cosPitch * cosRoll;

		_wind_measure.windspeed_north_f = wind_x_f * cosYaw*cosPitch + wind_y_f * (cosYaw*sinPitch*sinRoll - sinYaw*cosPitch) + wind_z_f * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
		_wind_measure.windspeed_east_f = wind_x_f * sinYaw*cosPitch + wind_y_f * (sinYaw*sinPitch*sinRoll + cosYaw*cosPitch) + wind_z_f * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);;
		_wind_measure.windspeed_down_f = -wind_x_f * sinPitch + wind_y_f * cosPitch*sinRoll + wind_z_f * cosPitch * cosRoll;

		if(fabsf(_solano_1.windspeed) <= 0.01f){
			_wind_measure.horiz_accuracy = -1.0f;
			_wind_measure.vert_accuracy = -1.0f;
		}
		else{
			_wind_measure.horiz_accuracy = 1.0f;
			_wind_measure.vert_accuracy = 1.0f;
		}

		_wind_measure.timestamp = _solano.timestamp;
		_wind_measure.timestamp_sample = _solano.timestamp_sample;

	}
	if(_param_sens_sol_multi.get()){
		_wind_measure.multiprobe = true;
	}
	else{
		_wind_measure.multiprobe = false; 
	}
	_previous_ts = _wind_measure.timestamp;
	_wind_measure_pub.publish(_wind_measure);

	perf_end(_loop_perf);
}

int FiveHolesProbe::task_spawn(int argc, char *argv[])
{
	FiveHolesProbe *instance = new FiveHolesProbe();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FiveHolesProbe::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	//print_message(_five_holes_probe);

	return 0;
}

int FiveHolesProbe::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FiveHolesProbe::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		This module publishes wind data from an embedded five holes probe.

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("five_holes_probe", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int five_holes_probe_main(int argc, char *argv[])
{
	return FiveHolesProbe::main(argc, argv);
}
