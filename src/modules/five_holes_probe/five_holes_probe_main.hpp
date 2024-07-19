/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/differential_pressure.h>
// #include <uORB/topics/euler_attitude.h>
#include <uORB/topics/solano.h>
#include <uORB/topics/wind_measure.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

/**
 * LiftDrag estimator app start / stop handling function
 */
extern "C" __EXPORT int five_holes_probe_main(int argc, char *argv[]);

class FiveHolesProbe : public ModuleBase<FiveHolesProbe>, public ModuleParams,
	public px4::WorkItem
{
public:
	FiveHolesProbe();

	virtual ~FiveHolesProbe();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * Check for parameter update and handle it.
	 */
	void parameter_update_poll();

	/**
	 * Compute the speed from a differential pressure
	 */
	 float compute_speed(float differential_pressure, float static_coefficient);
	 float compute_angle(float differential_pressure, float differential_pressure_center, float angle_coefficient);

	// Subs
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};		/**< parameter updates subscription */

	uORB::SubscriptionMultiArray<solano_s,3> _solano_subs{ORB_ID::solano};			/**< Solano probes subscriptions */

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this,ORB_ID(vehicle_attitude)};
	//uORB::Subscription _euler_attitude_sub{ORB_ID(euler_attitude)};			/**< euler angles subscription */
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};			/**< baro subscription */
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};			/**< position & drone speed subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< status subscription */

	// Pub
	uORB::Publication<wind_measure_s>		_wind_measure_pub;

	//Sub variables
	struct solano_s _solano {};
	struct solano_s _solano_sec {};
	struct solano_s _solano_third {};
	struct solano_s _solano_0 {};
	struct solano_s _solano_1 {};
	struct solano_s _solano_2 {};
	struct vehicle_air_data_s 			_vehicle_air_data {}; /**< baro */
	struct vehicle_attitude_s 	_vehicle_attitude {}; /**< attitude */
	struct vehicle_local_position_s 	_vehicle_local_position {}; /**< pos & speed */
	struct vehicle_status_s				_vehicle_status {};	/**< vehicle status */

	bool _probe0, _probe1, _probe2;
	float _vehicle_roll, _vehicle_pitch, _vehicle_yaw, _probe_roll, _probe_pitch, _probe_yaw;

	//Pub variables
	struct wind_measure_s				_wind_measure {};	/**< wind measurements */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	int _debug_cnt;
	float _rho;

	float _previous_ts;


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_SOL_ROLL>) _param_sens_sol_roll,
		(ParamFloat<px4::params::SENS_SOL_PITCH>) _param_sens_sol_pitch,
		(ParamFloat<px4::params::SENS_SOL_YAW>) _param_sens_sol_yaw,
		(ParamBool<px4::params::SENS_SOL_MULTI>) _param_sens_sol_multi
	)

};
