/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "GZMixingInterfaceESC.hpp"

#include <cmath>

bool GZMixingInterfaceESC::init(const std::string &model_name)
{

	// ESC feedback: /x500/command/motor_speed
	std::string motor_speed_topic = "/" + model_name + "/command/motor_speed";

	if (!_node.Subscribe(motor_speed_topic, &GZMixingInterfaceESC::motorSpeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", motor_speed_topic.c_str());
		return false;
	}

	// output eg /X500/command/motor_speed
	std::string actuator_topic = "/" + model_name + "/command/motor_speed";
	_actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);

	if (!_actuators_pub.Valid()) {
		PX4_ERR("failed to advertise %s", actuator_topic.c_str());
		return false;
	}

	_esc_status_pub.advertise();

	pthread_mutex_init(&_node_mutex, nullptr);

	ScheduleNow();

	return true;
}

bool GZMixingInterfaceESC::updateOutputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated)
{
	unsigned active_output_count = 0;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			active_output_count++;

		} else {
			break;
		}
	}

	if (active_output_count > 0) {

		// Guarda a saida legitima calculada pelo control allocation do PX4,
		// antes de qualquer injecao de ataque. E a unica fonte disponivel de
		// "comando correto" para restaurar caso a mitigacao detecte anomalia.
		uint16_t legitimate_outputs[MAX_ACTUATORS];

		for (unsigned i = 0; i < active_output_count; i++) {
			legitimate_outputs[i] = outputs[i];
		}

		gz::msgs::Actuators rotor_velocity_message;
		rotor_velocity_message.mutable_velocity()->Resize(active_output_count, 0);

		// ====== INJEÇÃO DO ATAQUE DE MOTORES UAVISEC ======
		_motor_attack.apply(outputs, active_output_count);
		// ==================================================

		// ====== MITIGACAO MOTOR - INICIO ======
		// Deteccao por implausibilidade fisica do comando prestes a ser
		// enviado, sem consultar _motor_attack_option/_motor_attack_index:
		//  (a) todos os motores comandados a empuxo proximo de zero enquanto
		//      o veiculo esta armado e nao pousado;
		//  (b) um motor com saida muito diferente da media dos demais,
		//      quando a media indica empuxo relevante.
		// (b) roda independente do estado de armado/pousado: um motor girando
		// muito diferente dos demais e implausivel em qualquer estado, inclusive
		// desarmado (o ataque nao respeita o estado de armado ao forcar o valor).
		bool anomaly_this_cycle = false;

		if (_motor_mitigation_enabled) {

			float sum_outputs = 0.0f;

			for (unsigned i = 0; i < active_output_count; i++) {
				sum_outputs += static_cast<float>(outputs[i]);
			}

			const float mean_output = sum_outputs / static_cast<float>(active_output_count);
			const bool all_near_zero = mean_output < MOTOR_MIN_ARMED_OUTPUT;

			if (all_near_zero) {
				actuator_armed_s armed{};
				const bool armed_available = _actuator_armed_sub.copy(&armed);
				const bool vehicle_armed = armed_available && armed.armed;

				vehicle_land_detected_s land_detected{};
				const bool land_detector_available = _vehicle_land_detected_sub.copy(&land_detected);
				const bool vehicle_not_landed = land_detector_available && !land_detected.landed;

				if (vehicle_armed && vehicle_not_landed) {
					// Restaura o comando legitimo em todos os motores ativos.
					for (unsigned i = 0; i < active_output_count; i++) {
						outputs[i] = legitimate_outputs[i];
					}

					anomaly_this_cycle = true;
				}

			} else {
				const float asymmetry_threshold = fmaxf(
					MOTOR_ASYMMETRY_MIN_ABS,
					mean_output * MOTOR_ASYMMETRY_FRACTION);

				for (unsigned i = 0; i < active_output_count; i++) {
					const float deviation = fabsf(static_cast<float>(outputs[i]) - mean_output);

					if (deviation > asymmetry_threshold) {
						outputs[i] = legitimate_outputs[i];
						anomaly_this_cycle = true;
					}
				}

				if (_motor_anomaly_active.load(std::memory_order_relaxed)) {
					static uint64_t motor_mitig_diag_last_us = 0;
					const uint64_t diag_now_us = hrt_absolute_time();

					if ((diag_now_us - motor_mitig_diag_last_us) > 1000000ULL) {
						motor_mitig_diag_last_us = diag_now_us;

						PX4_INFO("\n[Motor-Mitig] Status | mean=%.1f threshold=%.1f\n",
							static_cast<double>(mean_output),
							static_cast<double>(asymmetry_threshold));
					}
				}
			}
		}

		const uint64_t motor_mitig_timestamp = hrt_absolute_time();

		if (anomaly_this_cycle) {
			_motor_recovery_start_us = 0;

			if (_motor_anomaly_start_us == 0) {
				_motor_anomaly_start_us = motor_mitig_timestamp;
			}

			if (!_motor_anomaly_active.load(std::memory_order_relaxed) &&
					((motor_mitig_timestamp - _motor_anomaly_start_us) >= MOTOR_ANOMALY_CONFIRM_US)) {
				_motor_anomaly_active.store(true, std::memory_order_relaxed);
				PX4_WARN("\n[Motor-Mitig] Actuator command anomaly detected and rejected\n");
			}

		} else {
			_motor_anomaly_start_us = 0;

			if (_motor_anomaly_active.load(std::memory_order_relaxed)) {
				if (_motor_recovery_start_us == 0) {
					_motor_recovery_start_us = motor_mitig_timestamp;

				} else if ((motor_mitig_timestamp - _motor_recovery_start_us) >= MOTOR_RECOVERY_CONFIRM_US) {
					_motor_anomaly_active.store(false, std::memory_order_relaxed);
					_motor_recovery_start_us = 0;
					PX4_INFO("\n[Motor-Mitig] Actuator command anomaly cleared\n");
				}
			}
		}
		// ====== MITIGACAO MOTOR - FIM ======

		for (unsigned i = 0; i < active_output_count; i++) {
			rotor_velocity_message.set_velocity(i, static_cast<double>(outputs[i]));
		}

		if (_actuators_pub.Valid()) {
			return _actuators_pub.Publish(rotor_velocity_message);
		}
	}

	return false;
}

void GZMixingInterfaceESC::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

void GZMixingInterfaceESC::motorSpeedCallback(const gz::msgs::Actuators &actuators)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	esc_status_s esc_status{};
	int limited_escs = math::min(actuators.velocity_size(), (int)esc_status_s::CONNECTED_ESC_MAX);
	esc_status.esc_count = limited_escs;

	for (int i = 0; i < limited_escs; i++) {
		esc_status.esc[i].timestamp = hrt_absolute_time();
		esc_status.esc[i].esc_rpm = actuators.velocity(i);
		esc_status.esc_online_flags |= 1 << i;

		// This is a race condition with the failure detector, for smaller models it always resolves before
		// the failure detector runs, but for larger models (with more than 8 ESCs) the failure detector
		// can run before the velocity of some escs is set > 0. To mitigate this, if one esc has a velocity > 0,
		// we assume all escs are armed.
		if (actuators.velocity(i) > 0) {
			esc_status.esc_armed_flags = (1 << limited_escs) - 1;
		}
	}

	if (esc_status.esc_count > 0) {
		esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(esc_status);
	}

	pthread_mutex_unlock(&_node_mutex);
}