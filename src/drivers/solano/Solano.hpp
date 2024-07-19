/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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
 *
 * Driver for Sensirion Solano Differential Pressure Sensor
 *
 * Datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/8_Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP3x_Digital_Datasheet_V0.8.pdf
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/solano.h>

// #include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

//#include <lib/parameters/param.h>
//#include <uORB/topics/parameter_update.h>

#define I2C_ADDRESS_1_SOLANO 0x7A
#define I2C_ADDRESS_2_SOLANO 0x7B
#define I2C_ADDRESS_3_SOLANO 0x7C

static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface

#define Solano_SCALE_TEMPERATURE		200.0f
#define Solano_RESET_ADDR		0x00
#define Solano_RESET_CMD			0x06
#define Solano_CONT_MEAS_AVG_MODE	0x3615
#define Solano_CONT_MODE_STOP		0x3FF9

#define Solano_SCALE_PRESSURE_SDP31	60
#define Solano_SCALE_PRESSURE_SDP32	240
#define Solano_SCALE_PRESSURE_SDP33	20

// Measurement rate is 20Hz
#define Solano_MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / Solano_MEAS_RATE)	/* microseconds */

class Solano : public device::I2C, public I2CSPIDriver<Solano>
{
public:
	Solano(const I2CSPIDriverConfig &config);
	~Solano() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	enum class State {
		RequireConfig,
		Configuring,
		Running
	};

	int collect();

	int configure();
	int read_scale();

	bool init_solano();

	/**
	 * Calculate the CRC8 for the sensor payload data
	 */
	bool crc(const uint8_t data[], unsigned size, uint8_t checksum);

	/**
	 * Write a command in Sensirion specific logic
	 */
	int write_command(uint16_t command);
	uint8_t _id; 
	int _opened;
	uint16_t _scale{0};
	const bool _keep_retrying;
	State _state{State::RequireConfig};

	uORB::PublicationMulti<solano_s> _solano_pub{ORB_ID(solano)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};
