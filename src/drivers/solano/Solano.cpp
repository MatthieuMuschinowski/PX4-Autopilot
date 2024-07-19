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

#include "Solano.hpp"

using namespace time_literals;

Solano::Solano(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_keep_retrying(config.keep_running)
{
}

Solano::~Solano()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Solano::init()
{
	int ret = I2C::init();
	_opened=0;

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void Solano::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int Solano::probe()
{
	_retries = 1;
	bool require_initialization = !init_solano();

	if (require_initialization && _keep_retrying) {
		PX4_INFO("no sensor found, but will keep retrying");
		return 0;
	}

	return require_initialization ? -1 : 0;
}

// int Solano::write_command(uint16_t command)
// {
// 	uint8_t cmd[2];
// 	cmd[0] = static_cast<uint8_t>(command >> 8);
// 	cmd[1] = static_cast<uint8_t>(command & 0xff);
// 	return transfer(&cmd[0], 2, nullptr, 0);
// }

bool Solano::init_solano()
{
	return configure() == 0;
}

int Solano::configure()
{
	// int ret = write_command(Solano_CONT_MODE_STOP);
	//
	// if (ret == PX4_OK) {
	// 	px4_udelay(500); // Solano is unresponsive for 500us after stop continuous measurement command
	// 	ret = write_command(Solano_CONT_MEAS_AVG_MODE);
	// }
	//
	// if (ret != PX4_OK) {
	// 	perf_count(_comms_errors);
	// 	DEVICE_DEBUG("config failed");
	// 	_state = State::RequireConfig;
	// 	return ret;
	// }
	//
	// _state = State::Configuring;
	//
	// return ret;
	// Nothing to configure for us, return 0
	_state = State::Configuring;
	return 0;
}

int Solano::collect()
{
	solano_s solano{};
	perf_begin(_sample_perf);

	_opened++;
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// read 12 bytes from the sensor (used to be 6 in example)
	uint8_t val[12] {};
	int ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);

		//_solano_pub.publish(solano);
		return ret;
	}

	// // Check the CRC
	// if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5])) {
	// 	perf_count(_comms_errors);
	// 	return -EAGAIN;
	// }

	int16_t data_ws = (((int16_t)val[0]) << 8) | val[1];
	int16_t data_ws_f = (((int16_t)val[2]) << 8) | val[3];
	int16_t data_NS = (((int16_t)val[4]) << 8) | val[5];
	int16_t data_NS_f = (((int16_t)val[6]) << 8) | val[7];
	int16_t data_EW = (((int16_t)val[8]) << 8) | val[9];
	int16_t data_EW_f = (((int16_t)val[10]) << 8) | val[11];

	float ws = static_cast<float>(data_ws) / 100.0f;
	float ws_f = static_cast<float>(data_ws_f) / 100.0f;
	float angle_ns = static_cast<float>(data_NS) / 100.0f;
	float angle_ns_f = static_cast<float>(data_NS_f) / 100.0f;
	float angle_ew = static_cast<float>(data_EW) / 100.0f;
	float angle_ew_f = static_cast<float>(data_EW_f) / 100.0f;

	//float temperature_c = temp / static_cast<float>(Solano_SCALE_TEMPERATURE);

	solano.timestamp_sample = timestamp_sample;
	solano.windspeed = ws;
	solano.angle_ew = angle_ew;
	solano.angle_ns = angle_ns;

	solano.windspeed_f = ws_f;
	solano.angle_ew_f = angle_ew_f;
	solano.angle_ns_f = angle_ns_f;

	solano.timestamp = hrt_absolute_time();

	uint8_t address = get_device_address();
	if (address == I2C_ADDRESS_1_SOLANO){
		solano.id = 1;
	}
	else if (address == I2C_ADDRESS_2_SOLANO){
		solano.id = 2;
	}
	else if (address == I2C_ADDRESS_3_SOLANO){
		solano.id = 3;
	}
	else solano.id = 0;

	_solano_pub.publish(solano);

	perf_end(_sample_perf);

	return ret;
}

void Solano::RunImpl()
{
	switch (_state) {
	case State::RequireConfig:
		if (configure() == PX4_OK) {
			ScheduleDelayed(10_ms);

		} else {
			// periodically retry to configure
			ScheduleDelayed(300_ms);
		}

		break;

	case State::Configuring:
		_state = State::Running;

		ScheduleDelayed(10_ms);
		break;

	case State::Running:
		int ret = collect();

		if (ret != 0 && ret != -EAGAIN) {
			DEVICE_DEBUG("measure error");
			_state = State::RequireConfig;
		}

		ScheduleDelayed(CONVERSION_INTERVAL);
		break;
	}
}

bool Solano::crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc_value = 0xff;

	// calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
	for (unsigned i = 0; i < size; i++) {
		crc_value ^= (data[i]);

		for (int bit = 8; bit > 0; --bit) {
			if (crc_value & 0x80) {
				crc_value = (crc_value << 1) ^ 0x31;

			} else {
				crc_value = (crc_value << 1);
			}
		}
	}

	// verify checksum
	return (crc_value == checksum);
}
