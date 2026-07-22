/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/mixer_module/mixer_module.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <atomic>


// GZBridge mixing class for ESCs.
// It is separate from GZBridge to have separate WorkItems and therefore allowing independent scheduling
// All work items are expected to run on the same work queue.
class GZMixingInterfaceESC : public OutputModuleInterface
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;

	GZMixingInterfaceESC(gz::transport::Node &node) :
		OutputModuleInterface(MODULE_NAME "-actuators-esc", px4::wq_configurations::rate_ctrl),
		_node(node)
	{}

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated) override;

	MixingOutput &mixingOutput() { return _mixing_output; }

	bool init(const std::string &model_name);

	void stop()
	{
		_mixing_output.unregister();
		ScheduleClear();
	}

	// ====== ADIÇÃO UAVISEC ======
	void setMotorAttack(int option, int index, double speed) {
		_motor_attack_option = option;
		_motor_attack_index = index;
		_motor_attack_speed = speed;
	}
	// ============================

	// ====== MITIGACAO MOTOR ======
	// Habilita/desabilita a mitigacao. Chamado pelo GZBridge a partir do mesmo
	// topico unico de mitigacao usado por GPS e IMU.
	void setMitigationEnabled(bool enabled) {
		_motor_mitigation_enabled = enabled;
		_motor_anomaly_active.store(false);
		_motor_anomaly_start_us = 0;
		_motor_recovery_start_us = 0;
	}

	// Consultado pelo GZBridge (thread diferente) para acionar o pouso
	// automatico quando uma anomalia de atuacao esta confirmada.
	bool motorAnomalyActive() const {
		return _motor_anomaly_active.load(std::memory_order_relaxed);
	}
	// ==============================

private:
	friend class GZBridge;

	void Run() override;

	void motorSpeedCallback(const gz::msgs::Actuators &actuators);

	gz::transport::Node &_node;
	pthread_mutex_t _node_mutex;

	// ====== VARIÁVEIS UAVISEC ======
	int _motor_attack_option{0};
	int _motor_attack_index{0};
	double _motor_attack_speed{0.0};
	// ===============================

	// ====== MITIGACAO MOTOR - ESTADO ======
	bool _motor_mitigation_enabled{false};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// Lido pelo GZBridge em outra work queue/thread.
	std::atomic<bool> _motor_anomaly_active{false};
	uint64_t _motor_anomaly_start_us{0};
	uint64_t _motor_recovery_start_us{0};

	// Abaixo deste valor de saida (na mesma unidade de _motor_attack_speed,
	// faixa 0-1450), o motor e considerado "sem empuxo relevante".
	static constexpr float MOTOR_MIN_ARMED_OUTPUT = 50.0f;
	// Desvio maximo tolerado entre a saida de um motor e a media dos demais,
	// como fracao da propria media.
	static constexpr float MOTOR_ASYMMETRY_FRACTION = 0.20f; // 20%
	// Piso absoluto para a media nao gerar um limiar irrisorio perto do solo.
	static constexpr float MOTOR_ASYMMETRY_MIN_ABS = 60.0f;
	// Tempo minimo sustentado para confirmar a anomalia/recuperacao antes de
	// sinalizar ao GZBridge (o bloqueio do comando em si e imediato, sem essa espera).
	static constexpr uint64_t MOTOR_ANOMALY_CONFIRM_US = 100000ULL;  // 100 ms
	static constexpr uint64_t MOTOR_RECOVERY_CONFIRM_US = 300000ULL; // 300 ms
	// ========================================

	MixingOutput _mixing_output{"SIM_GZ_EC", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	gz::transport::Node::Publisher _actuators_pub;

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

};