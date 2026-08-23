/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
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

#include "GZBridge.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

#include <lib/atmosphere/atmosphere.h>
#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/getopt.h>

#include <iostream>
#include <string>
#include <cstring>


// Inicializa a instancia da ponte Gazebo-PX4 com o mundo e o modelo simulados.
GZBridge::GZBridge(const std::string &world, const std::string &model_name) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _world_name(world),
    _model_name(model_name)
{
    updateParams();
}

// Cancela as inscricoes do no Gazebo ao destruir a ponte.
GZBridge::~GZBridge()
{
    for (auto &sub_topic : _node.SubscribedTopics()) {
        _node.Unsubscribe(sub_topic);
    }
}

// Inicializa sensores, topicos de ataque, interfaces de atuadores e agendamento do modulo.
int GZBridge::init()
{

    // O relogio e obrigatorio para evitar timestamps invalidos no EKF.
    if (!subscribeClock(true)) {
        return PX4_ERROR;
    }

    while (1) {
        px4_usleep(25000);

        if (_realtime_clock_set) {
            px4_usleep(25000);
            break;
        }
    }

    if (!subscribePoseInfo(true)) {
        return PX4_ERROR;
    }

    if (!subscribeImu(true)) {
        return PX4_ERROR;
    }

    if (!subscribeMag(true)) {
        return PX4_ERROR;
    }

    // Sensores opcionais sao inscritos apenas quando habilitados por parametro.
    if (_sim_gz_en_gps.get()) {
        if (!subscribeNavsat(false)) {
            return PX4_ERROR;
        }
    }

    if (_sim_gz_en_baro.get()) {
        if (!subscribeAirPressure(false)) {
            return PX4_ERROR;
        }
    }

    if (_sim_gz_en_lidar.get()) {
        if (!subscribeDistanceSensor(false)) {
            return PX4_ERROR;
        }

        // LiDAR frontal em modelos combinados (link/sensor com nome
        // distinto do down). Opcional: modelos com um so sensor de
        // distancia nao tem esse topico, e a falha aqui e esperada.
        subscribeDistanceSensorFront(false);

        // Camera de profundidade e opcional: so o x500_uavjamsim a tem,
        // usada como referencia cruzada do LiDAR frontal. Falha em se
        // inscrever nao interrompe a inicializacao.
        _lidar_mitigation.init();
    }

    if (_sim_gz_en_aspd.get()) {
        if (!subscribeAirspeed(false)) {
            return PX4_ERROR;
        }
    }

    if (_sim_gz_en_flow.get()) {
        if (!subscribeOpticalFlow(false)) {
            return PX4_ERROR;
        }
    }

    if (_sim_gz_en_odom.get()) {
        if (!subscribeOdometry(false)) {
            return PX4_ERROR;
        }
    }

    if (_sim_gz_en_lidar.get()) {
        if (!subscribeLaserScan(false)) {
            return PX4_ERROR;
        }
    }

    // Topicos UAVJamSim permitem ativar ataques e mitigacao durante a simulacao.
    if (!subscribeAttacks(false)) {
        return PX4_ERROR;
    }

    // Publicador auxiliar usado para encaminhar comandos ao plugin de camera.
    _stream_cmd_pub = _node.Advertise<gz::msgs::Int32>("/gazebo/default/attack/stream_cmd");

    if (!_mixing_interface_esc.init(_model_name)) {
        PX4_ERR("failed to init ESC output");
        return PX4_ERROR;
    }

    if (!_mixing_interface_servo.init(_model_name)) {
        PX4_ERR("failed to init servo output");
        return PX4_ERROR;
    }

    if (!_mixing_interface_wheel.init(_model_name)) {
        PX4_ERR("failed to init motor output");
        return PX4_ERROR;
    }

#if defined(CONFIG_MODULES_GIMBAL)

    if (!_gimbal.init(_world_name, _model_name)) {
        PX4_ERR("failed to init gimbal");
        return PX4_ERROR;
    }

#endif

    ScheduleNow();
    return OK;
}

// Executa o ciclo periodico do modulo e atualiza parametros/interfaces de saida.
void GZBridge::Run()
{
    if (should_exit()) {
        ScheduleClear();

        _mixing_interface_esc.stop();
        _mixing_interface_servo.stop();
        _mixing_interface_wheel.stop();
        _gimbal.stop();

        exit_and_cleanup();
        return;
    }

    if (_parameter_update_sub.updated()) {
        parameter_update_s pupdate;
        _parameter_update_sub.copy(&pupdate);

        updateParams();

        _mixing_interface_esc.updateParams();
        _mixing_interface_servo.updateParams();
        _mixing_interface_wheel.updateParams();
        _gimbal.updateParams();
    }

    ScheduleDelayed(10_ms);
}

// Inscreve o modulo no relogio do Gazebo para sincronizar o tempo do PX4 SITL.
bool GZBridge::subscribeClock(bool required)
{
    std::string clock_topic = "/world/" + _world_name + "/clock";

    if (!_node.Subscribe(clock_topic, &GZBridge::clockCallback, this)) {
        PX4_ERR("failed to subscribe to %s", clock_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no topico de pose do modelo para publicar estados simulados auxiliares no PX4.
bool GZBridge::subscribePoseInfo(bool required)
{
    std::string world_pose_topic = "/world/" + _world_name + "/pose/info";

    if (!_node.Subscribe(world_pose_topic, &GZBridge::poseInfoCallback, this)) {
        PX4_ERR("failed to subscribe to %s", world_pose_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no sensor IMU simulado.
bool GZBridge::subscribeImu(bool required)
{
    std::string imu_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/imu_sensor/imu";

    if (!_node.Subscribe(imu_topic, &GZBridge::imuCallback, this)) {
        PX4_ERR("failed to subscribe to %s", imu_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no magnetometro simulado.
bool GZBridge::subscribeMag(bool required)
{
    std::string mag_topic = "/world/" + _world_name + "/model/" + _model_name +
                "/link/base_link/sensor/magnetometer_sensor/magnetometer";

    if (!_node.Subscribe(mag_topic, &GZBridge::magnetometerCallback, this)) {
        PX4_ERR("failed to subscribe to %s", mag_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte na odometria com covariancia do Gazebo.
bool GZBridge::subscribeOdometry(bool required)
{

    std::string odometry_topic = "/model/" + _model_name + "/odometry_with_covariance";

    if (!_node.Subscribe(odometry_topic, &GZBridge::odometryCallback, this)) {
        PX4_ERR("failed to subscribe to %s", odometry_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no LiDAR 2D usado para prevencao de colisao.
bool GZBridge::subscribeLaserScan(bool required)
{
    std::string laser_scan_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/link/sensor/lidar_2d_v2/scan";

    if (!_node.Subscribe(laser_scan_topic, &GZBridge::laserScanCallback, this)) {
        PX4_WARN("failed to subscribe to %s", laser_scan_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no sensor de distancia LiDAR.
bool GZBridge::subscribeDistanceSensor(bool required)
{
    std::string lidar_sensor = "/world/" + _world_name + "/model/" + _model_name +
                   "/link/lidar_sensor_link/sensor/lidar/scan";

    if (!_node.Subscribe(lidar_sensor, &GZBridge::laserScantoLidarSensorCallback, this)) {
        PX4_WARN("failed to subscribe to %s", lidar_sensor.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no LiDAR frontal de modelos combinados (link/sensor com
// nome distinto do down, ex.: x500_uavjamsim). Reaproveita o mesmo callback
// do sensor de distancia: a orientacao e detectada dinamicamente pelo
// quaternion da mensagem, e o publish final escolhe a instancia certa
// (_distance_sensor_pub ou _distance_sensor_front_pub) de acordo com ela.
bool GZBridge::subscribeDistanceSensorFront(bool required)
{
    std::string lidar_front_sensor = "/world/" + _world_name + "/model/" + _model_name +
                   "/link/lidar_front_sensor_link/sensor/lidar_front/scan";

    if (!_node.Subscribe(lidar_front_sensor, &GZBridge::laserScantoLidarSensorFrontCallback, this)) {
        PX4_WARN("failed to subscribe to %s", lidar_front_sensor.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no sensor de velocidade do ar.
bool GZBridge::subscribeAirspeed(bool required)
{
    std::string airspeed_topic = "/world/" + _world_name + "/model/" + _model_name +
                     "/link/airspeed_link/sensor/air_speed/air_speed";

    if (!_node.Subscribe(airspeed_topic, &GZBridge::airspeedCallback, this)) {
        PX4_ERR("failed to subscribe to %s", airspeed_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no barometro simulado.
bool GZBridge::subscribeAirPressure(bool required)
{
    std::string air_pressure_topic = "/world/" + _world_name + "/model/" + _model_name +
                     "/link/base_link/sensor/air_pressure_sensor/air_pressure";

    if (!_node.Subscribe(air_pressure_topic, &GZBridge::airPressureCallback, this)) {
        PX4_ERR("failed to subscribe to %s", air_pressure_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no sensor GPS/GNSS simulado.
bool GZBridge::subscribeNavsat(bool required)
{
    std::string nav_sat_topic = "/world/" + _world_name + "/model/" + _model_name +
                    "/link/base_link/sensor/navsat_sensor/navsat";

    if (!_node.Subscribe(nav_sat_topic, &GZBridge::navSatCallback, this)) {
        PX4_ERR("failed to subscribe to %s", nav_sat_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// Inscreve a ponte no sensor de fluxo optico simulado.
bool GZBridge::subscribeOpticalFlow(bool required)
{
    std::string flow_topic = "/world/" + _world_name + "/model/" + _model_name +
                 "/link/flow_link/sensor/optical_flow/optical_flow";

    if (!_node.Subscribe(flow_topic, &GZBridge::opticalFlowCallback, this)) {
        PX4_ERR("failed to subscribe to %s", flow_topic.c_str());
        return required ? false : true;
    }

    return true;
}

// ======== TOPICOS DE ATAQUE E MITIGACAO - INICIO ========
// Inscreve a ponte nos topicos de comando dos ataques UAVJamSim e da mitigacao.
bool GZBridge::subscribeAttacks(bool required)
{
    // Topicos de ataque usados pelo spoofer para alterar sensores/atuadores em tempo de execucao.
    std::string gps_attack_topic = "/gazebo/default/attack/gps";
    std::string gps_rot_attack_topic = "/gazebo/default/attack/gps_rot";
    std::string imu_attack_topic = "/gazebo/default/attack/imu";
    std::string motor_attack_topic = "/gazebo/default/attack/motor";
    std::string mag_attack_topic = "/gazebo/default/attack/mag";
    std::string lidar_attack_topic = "/gazebo/default/attack/lidar";
    std::string stream_attack_topic = "/gazebo/default/attack/stream";
    std::string baro_attack_topic = "/gazebo/default/attack/baro";
    std::string jamming_attack_topic = "/gazebo/default/attack/jamming";

    // Topico separado para habilitar ou desabilitar a mitigacao GPS.
    std::string attack_mitigation_topic = "/gazebo/default/mitigation/control";

    // Sinal publicado pelo GstCameraSystem (processo separado do Gazebo)
    // quando a mitigacao de flip do stream confirma um ataque. O GZBridge
    // so repassa o aviso para o terminal do PX4, ja que o plugin de camera
    // nao tem acesso a PX4_WARN nem a esse console.
    std::string stream_flip_detected_topic = "/gazebo/default/attack/stream_flip_detected";

    // Mesmo mecanismo, para a mitigacao do quadrado preto do stream.
    std::string stream_black_detected_topic = "/gazebo/default/attack/stream_black_detected";

    if (!_gps_offset_attack.init(gps_attack_topic)) {
        return required ? false : true;
    }

    if (!_gps_rot_attack.init(gps_rot_attack_topic)) {
        return required ? false : true;
    }

    if (!_imu_attack.init(imu_attack_topic)) {
        return required ? false : true;
    }

    if (!_node.Subscribe(motor_attack_topic, &GZBridge::motorAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", motor_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_mag_attack.init(mag_attack_topic)) {
        return required ? false : true;
    }

    if (!_lidar_attack.init(lidar_attack_topic)) {
        return required ? false : true;
    }

    if (!_node.Subscribe(stream_attack_topic, &GZBridge::streamAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", stream_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_baro_attack.init(baro_attack_topic)) {
        return required ? false : true;
    }

    if (!_jamming_attack.init(jamming_attack_topic)) {
        return required ? false : true;
    }

    if (!_node.Subscribe(attack_mitigation_topic, &GZBridge::attackMitigationCallback, this)) {
        PX4_ERR("failed to subscribe to mitigation topic: %s", attack_mitigation_topic.c_str());
        return required ? false : true;
    }

    if (!_node.Subscribe(stream_flip_detected_topic, &GZBridge::streamFlipDetectedCallback, this)) {
        PX4_WARN("failed to subscribe to %s", stream_flip_detected_topic.c_str());
    }

    if (!_node.Subscribe(stream_black_detected_topic, &GZBridge::streamBlackDetectedCallback, this)) {
        PX4_WARN("failed to subscribe to %s", stream_black_detected_topic.c_str());
    }

    return true;
}
// ======== TOPICOS DE ATAQUE E MITIGACAO - FIM ========

// ======== ATAQUE STREAM DE CAMERA - INICIO ========
// Ataque no stream de camera: traduz o comando UAVJamSim para o plugin de camera do Gazebo.
void GZBridge::streamAttackCallback(const gz::msgs::Int32 &msg)
{

    // 1: inversao de buffer; 2: ruido/mascara; 0: desativado.
    _stream_attack_option = msg.data();
    PX4_INFO("Stream Attack: Option=%d", _stream_attack_option);

    // O plugin de camera recebe Int32; por isso o comando e encaminhado nesse formato.
    gz::msgs::Int32 stream_cmd;

    if (_stream_attack_option == 1) {
        stream_cmd.set_data(1);
    }
    else if (_stream_attack_option == 2) {
        stream_cmd.set_data(2);
    }
    else {
        stream_cmd.set_data(0);
    }

    _stream_cmd_pub.Publish(stream_cmd);
}
// ======== ATAQUE STREAM DE CAMERA - FIM ========

// ======== MITIGACAO FLIP - INICIO ========
// So repassa o aviso ja emitido pelo GstCameraSystem (processo do Gazebo,
// sem acesso a este terminal) no formato usado pelas demais mitigacoes.
// A deteccao em si roda inteiramente no plugin de camera, por continuidade
// temporal do proprio video — este callback nao decide nada, so imprime.
void GZBridge::streamFlipDetectedCallback(const gz::msgs::Int32 & /*msg*/)
{
    PX4_WARN("\n[Flip-Mitig] Attack detected: FLIP\n");
}
// ======== MITIGACAO FLIP - FIM ========

// ======== MITIGACAO QUADRADO PRETO - INICIO ========
// Mesmo mecanismo do flip: so repassa o aviso ja emitido pelo
// GstCameraSystem, sem decidir nada aqui.
void GZBridge::streamBlackDetectedCallback(const gz::msgs::Int32 & /*msg*/)
{
    PX4_WARN("\n[Black-Mitig] Attack detected: BLACK_SQUARE\n");
}
// ======== MITIGACAO QUADRADO PRETO - FIM ========

// ======== ATAQUE MOTOR - INICIO ========
// Ataque em motor: repassa opcao, indice do motor e velocidade para a interface de mistura dos ESCs.
void GZBridge::motorAttackCallback(const gz::msgs::Vector3d &msg)
{
    int option = static_cast<int>(msg.x());
    int index = static_cast<int>(msg.y());
    double speed = msg.z();

    // A alteracao efetiva do motor ocorre dentro da interface de mistura dos ESCs.
    _mixing_interface_esc.setMotorAttack(option, index, speed);
}
// ======== ATAQUE MOTOR - FIM ========

// ======== MITIGACAO GPS - INICIO ========
// Controle da mitigacao GPS: liga/desliga o filtro de rejeicao e reinicia o estado da ancora.
void GZBridge::attackMitigationCallback(const gz::msgs::Vector3d &msg)
{
    // msg.x = 1 ativa a mitigacao; qualquer outro valor desativa.
    const bool requested_state = (static_cast<int>(msg.x()) == 1);

    if (requested_state == _attack_mitigation_enabled) {
        return;
    }

    // Ao mudar o estado, reinicia a ancora e o filtro interno da mitigacao.
    _attack_mitigation_enabled = requested_state;

    _jmit_lpos_alt_msl = 0.0;
    _jmit_lpos_alt_valid = false;
    _jmit_lpos_timestamp = 0;
    _ground_distance_m = NAN;
    _ground_distance_valid = false;
    _ground_distance_timestamp = 0;
    _sim_agl_m = NAN;
    _sim_agl_valid = false;
    _sim_agl_timestamp = 0;
    _sim_roll_rad = NAN;
    _sim_pitch_rad = NAN;
    _sim_att_valid = false;
    _sim_att_timestamp = 0;

    _gps_mitigation.reset();
    _lpos_heading_rad = 0.0f;

    // Reinicia o estado da mitigacao de IMU (deteccao, linha de base, pouso).
    _imu_mitigation.reset();

    // Reinicia o estado de pouso/desarme da mitigacao de motor. A deteccao e
    // a correcao em si vivem em GZMixingInterfaceESC, avisadas via setMitigationEnabled.
    _motor_landing_active = false;
    _motor_auto_land_sent = false;
    _motor_auto_disarm_sent = false;
    _motor_auto_land_arrival_us = 0;
    _motor_auto_land_last_arrived_us = 0;
    _motor_land_hold_n_m = 0.0;
    _motor_land_hold_e_m = 0.0;
    _motor_sim_agl_ground_count = 0;
    _mixing_interface_esc.setMitigationEnabled(_attack_mitigation_enabled);

    // Reinicia o estado da mitigacao de magnetometro (deteccao, linha de
    // base, pouso). O cache de atitude da VIO nao e resetado (ver
    // comentario em MagnetometerMitigation::reset()).
    _mag_mitigation.reset();

    // Reinicia o estado das mitigacoes de LiDAR (down/front/2D): deteccao,
    // ancora, pouso. O cache da camera de profundidade nao e resetado (ver
    // comentario em LidarMitigation::reset()).
    _lidar_mitigation.reset();

    // Reinicia o estado da mitigacao de barometro (deteccao, ancora, pouso).
    _baro_baseline_initialized = false;
    _baro_baseline_warmup_count = 0;
    _baro_anchor_offset_m = 0.0;
    _baro_anomaly_active = false;
    _baro_anomaly_start_us = 0;
    _baro_recovery_start_us = 0;
    _baro_mitig_diag_count = 0;
    _baro_landing_active = false;
    _baro_auto_land_sent = false;
    _baro_auto_disarm_sent = false;
    _baro_auto_land_arrival_us = 0;
    _baro_auto_land_last_arrived_us = 0;
    _baro_land_hold_n_m = 0.0;
    _baro_land_hold_e_m = 0.0;
    _baro_sim_agl_ground_count = 0;

    if (_attack_mitigation_enabled) {
        PX4_WARN("\n[Mitigation] KF-Jamming mitigation ENABLED\n");
    } else {
        PX4_INFO("\n[Mitigation] KF-Jamming mitigation DISABLED\n");
    }
}
// ======== MITIGACAO GPS - FIM ========

// Sincroniza o relogio do PX4 SITL com o tempo de simulacao do Gazebo.
void GZBridge::clockCallback(const gz::msgs::Clock &msg)
{

    // O tempo simulado e propagado para os relogios usados pelo PX4.
    struct timespec ts;
    ts.tv_sec = msg.sim().sec();
    ts.tv_nsec = msg.sim().nsec();

    if (!_realtime_clock_set) {

        px4_clock_settime(CLOCK_REALTIME, &ts);
        _realtime_clock_set = true;

    } else {

        px4_clock_settime(CLOCK_MONOTONIC, &ts);
    }
}

// Publica o fluxo optico simulado no formato uORB esperado pelo PX4.
void GZBridge::opticalFlowCallback(const px4::msgs::OpticalFlow &msg)
{
    // Converte a mensagem Gazebo para sensor_optical_flow_s.
    sensor_optical_flow_s report = {};

    report.timestamp = hrt_absolute_time();
    report.timestamp_sample = msg.time_usec();
    report.pixel_flow[0] = msg.integrated_x();
    report.pixel_flow[1] = msg.integrated_y();
    report.quality = msg.quality();
    report.integration_timespan_us = msg.integration_time_us();

    device::Device::DeviceId id;
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.bus = 0;
    id.devid_s.address = 0;
    id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;
    report.device_id = id.devid;

    report.mode = sensor_optical_flow_s::MODE_LOWLIGHT;
    report.max_flow_rate = 7.4f;
    report.min_ground_distance = 0.f;
    report.max_ground_distance = 30.f;
    report.error_count = 0;

    _optical_flow_pub.publish(report);
}

// Publica o magnetometro e aplica o ataque de troca de eixos quando configurado.
void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    // Montagem da mensagem sensor_gps publicada para o PX4.
    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_MAG_DEVTYPE_MAGSIM;
    id.devid_s.bus = 1;
    id.devid_s.address = 3;

    sensor_mag_s report{};
    report.timestamp = timestamp;
    report.timestamp_sample = timestamp;
    report.device_id = id.devid;
    report.temperature = this->_temperature;

    // Conversao do campo magnetico do referencial Gazebo para o referencial PX4.
    float raw_x = -msg.field_tesla().y();
    float raw_y = -msg.field_tesla().x();
    float raw_z = msg.field_tesla().z();

    // Ataque de magnetometro: opcao 1 troca os eixos X e Y.
    _mag_attack.apply(raw_x, raw_y, raw_z, report.x, report.y, report.z);

    // ======== MITIGACAO MAGNETOMETRO - INICIO ========
    _mag_mitigation.detectAndCorrect(_attack_mitigation_enabled, timestamp, report.x, report.y);
    // ======== MITIGACAO MAGNETOMETRO - FIM ========

    _sensor_mag_pub.publish(report);
}

// Publica o barometro, aplica ataque barometrico e mantem cache de altitude para a mitigacao GPS.
void GZBridge::airPressureCallback(const gz::msgs::FluidPressure &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_BARO_DEVTYPE_BAROSIM;
    id.devid_s.bus = 1;
    id.devid_s.address = 1;

    sensor_baro_s report{};
    report.timestamp = timestamp;
    report.timestamp_sample = timestamp;
    report.device_id = id.devid;

    // Leitura nominal do barometro antes da injecao do ataque.
    float raw_pressure = msg.pressure();
    float raw_temperature = this->_temperature;

    // Ataque barometrico: converte offset de altitude em alteracao fisica de pressao.
    _baro_attack.apply(raw_pressure, raw_temperature, report.pressure, report.temperature);

    // ======== MITIGACAO BAROMETRO - INICIO ========
    // Deteccao de anomalia na altitude implicita na pressao publicada, sem
    // consultar _baro_attack_option: divergencia contra o valor esperado
    // por uma ancora entre essa altitude e a altitude do GPS (referencia
    // independente), calibrada a partir das primeiras amostras saudaveis.
    // O ataque e um offset constante de altitude, que uma comparacao por
    // taxa de variacao nunca pegaria (o offset se cancela na diferenca
    // entre amostras): so uma comparacao por valor absoluto contra essa
    // ancora detecta.
    if (_attack_mitigation_enabled) {

        constexpr float P0 = 101325.0f;
        constexpr float T0 = 288.15f;
        constexpr float L  = 0.0065f;
        constexpr float EXP = 0.190263f;

        const float P_published = report.pressure;
        const bool pressure_valid =
            PX4_ISFINITE(P_published) && P_published > 1000.0f && P_published < 120000.0f;

        double baro_alt_from_report = NAN;

        if (pressure_valid) {
            baro_alt_from_report =
                (static_cast<double>(T0) / static_cast<double>(L)) *
                (1.0 - pow(static_cast<double>(P_published) / static_cast<double>(P0), static_cast<double>(EXP)));
        }

        const bool gps_alt_recent =
            _gps_real_valid &&
            (timestamp >= _gps_real_alt_timestamp) &&
            ((timestamp - _gps_real_alt_timestamp) <= BARO_GPS_ALT_MAX_AGE_US) &&
            PX4_ISFINITE(_gps_real_alt);

        double residual = 0.0;
        bool sample_anomalous = false;

        if (gps_alt_recent && pressure_valid && _baro_baseline_initialized) {
            const double expected_alt = _baro_anchor_offset_m + _gps_real_alt;
            residual = baro_alt_from_report - expected_alt;
            sample_anomalous = fabs(residual) > BARO_RESIDUAL_THRESHOLD_M;
        }

        if (!sample_anomalous) {

            // Atualiza a ancora apenas com amostras saudaveis. As primeiras
            // amostras (warmup) calibram com um alpha mais agressivo;
            // depois, adapta lentamente (tolera deriva legitima, mas um
            // ataque sustentado nao chega a ser absorvido, pois so entra
            // aqui quando a amostra nao e anomala).
            if (gps_alt_recent && pressure_valid) {
                const double sample_offset = baro_alt_from_report - _gps_real_alt;

                if (_baro_baseline_warmup_count == 0) {
                    _baro_anchor_offset_m = sample_offset;
                    _baro_baseline_warmup_count = 1;

                } else if (_baro_baseline_warmup_count < BARO_BASELINE_WARMUP_SAMPLES) {
                    _baro_anchor_offset_m += BARO_BASELINE_WARMUP_ALPHA * (sample_offset - _baro_anchor_offset_m);
                    _baro_baseline_warmup_count++;

                    if (_baro_baseline_warmup_count >= BARO_BASELINE_WARMUP_SAMPLES) {
                        _baro_baseline_initialized = true;
                    }

                } else {
                    _baro_anchor_offset_m += BARO_BASELINE_ALPHA * (sample_offset - _baro_anchor_offset_m);
                }
            }

            if (_baro_anomaly_active) {
                if (_baro_recovery_start_us == 0) {
                    _baro_recovery_start_us = timestamp;

                } else if ((timestamp - _baro_recovery_start_us) >= BARO_RECOVERY_CONFIRM_US) {
                    _baro_anomaly_active = false;
                    _baro_anomaly_start_us = 0;
                    _baro_mitig_diag_count = 0;

                    PX4_INFO("\n[Baro-Mitig] Sensor anomaly cleared\n");
                }
            }

        } else {
            _baro_recovery_start_us = 0;

            if (_baro_anomaly_start_us == 0) {
                _baro_anomaly_start_us = timestamp;
            }

            if (!_baro_anomaly_active &&
                    ((timestamp - _baro_anomaly_start_us) >= BARO_ANOMALY_CONFIRM_US)) {
                _baro_anomaly_active = true;

                PX4_WARN("\n[Baro-Mitig] Sensor anomaly detected | alt=%.2f m | residual=%.2f m\n",
                    baro_alt_from_report, residual);
            }

            // Reconstroi a pressao a partir da altitude esperada (ancora +
            // altitude atual do GPS), usando o mesmo modelo de atmosfera
            // padrao ja usado para calcular altitude a partir de pressao.
            // A temperatura publicada nunca precisa ser reconstruida: vem
            // de this->_temperature (airspeedCallback), caminho que o
            // ataque de barometro nunca toca.
            if (_baro_anomaly_active && gps_alt_recent && _baro_baseline_initialized) {
                const double expected_alt = _baro_anchor_offset_m + _gps_real_alt;
                const double corrected_pressure =
                    static_cast<double>(P0) *
                    pow(1.0 - (static_cast<double>(L) * expected_alt) / static_cast<double>(T0),
                        1.0 / static_cast<double>(EXP));

                if (PX4_ISFINITE(corrected_pressure) && corrected_pressure > 1000.0 && corrected_pressure < 120000.0) {
                    report.pressure = static_cast<float>(corrected_pressure);
                }

                report.temperature = raw_temperature;
            }

            if (_baro_anomaly_active && (_baro_mitig_diag_count < BARO_MITIG_DIAG_MAX_COUNT)) {
                static uint64_t baro_mitig_diag_last_us = 0;

                if ((timestamp - baro_mitig_diag_last_us) > 1000000ULL) {
                    baro_mitig_diag_last_us = timestamp;
                    _baro_mitig_diag_count++;

                    PX4_INFO("\n[Baro-Mitig] Status | pressure=%.1f Pa | residual=%.2f m | gps_alt=%s\n",
                        static_cast<double>(report.pressure),
                        residual,
                        gps_alt_recent ? "ok" : "stale");

                    if (_baro_mitig_diag_count == BARO_MITIG_DIAG_MAX_COUNT) {
                        PX4_WARN("\n[Baro-Mitig] Check: listener sensor_baro\n");
                    }
                }
            }
        }
    }
    // ======== MITIGACAO BAROMETRO - FIM ========

    _sensor_baro_pub.publish(report);

    // ======== MITIGACAO GPS - INICIO ========
    // Cache usado pela mitigacao quando o barometro esta em estado
    // confiavel, decidido pela propria deteccao de anomalia do barometro
    // (sintoma), nao pela flag de comando do ataque.
    if (!_baro_anomaly_active) {
        constexpr float P0 = 101325.0f;
        constexpr float T0 = 288.15f;
        constexpr float L  = 0.0065f;
        constexpr float EXP = 0.190263f;

        const float P = report.pressure;

        if (PX4_ISFINITE(P) && P > 1000.0f && P < 120000.0f) {
            _jmit_baro_alt_m =
                (T0 / L) * (1.0f - powf(P / P0, EXP));

            _jmit_baro_valid = true;
            _jmit_baro_timestamp = timestamp;
        }
    }
    // ======== MITIGACAO GPS - FIM ========

}

// Publica a velocidade do ar e atualiza a temperatura usada pelo barometro.
void GZBridge::airspeedCallback(const gz::msgs::AirSpeed &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;
    id.devid_s.bus = 1;
    id.devid_s.address = 1;

    // Publicacao no formato differential_pressure_s consumido pelo PX4.
    differential_pressure_s report{};
    report.timestamp = timestamp;
    report.timestamp_sample = timestamp;
    report.device_id = id.devid;
    report.differential_pressure_pa = msg.diff_pressure();
    report.temperature = static_cast<float>(msg.temperature()) + atmosphere::kAbsoluteNullCelsius;
    _differential_pressure_pub.publish(report);

    this->_temperature = report.temperature;
}

// Monta o contexto compartilhado usado por GpsMitigation a partir dos caches
// ja mantidos pelo GZBridge (VIO, LPOS, atitude/AGL, distancia, barometro).
GpsMitigationContext GZBridge::buildGpsMitigationContext() const
{
    GpsMitigationContext ctx{};

    ctx.vio_valid = _vio_valid;
    ctx.vio_timestamp = _vio_timestamp;
    ctx.vio_n_m = _vio_n_m;
    ctx.vio_e_m = _vio_e_m;
    ctx.vio_alt_msl = _vio_alt_msl;
    ctx.vio_vel_n = _vio_vel_n;
    ctx.vio_vel_e = _vio_vel_e;
    ctx.vio_vel_d = _vio_vel_d;

    ctx.lpos_xy_valid = _lpos_xy_valid;
    ctx.lpos_z_valid = _lpos_z_valid;
    ctx.lpos_timestamp = _lpos_timestamp;
    ctx.lpos_n_m = _lpos_n_m;
    ctx.lpos_e_m = _lpos_e_m;
    ctx.lpos_alt_msl = _lpos_alt_msl;
    ctx.imu_dr_vel_n = _imu_dr_vel_n;
    ctx.imu_dr_vel_e = _imu_dr_vel_e;
    ctx.imu_dr_vel_d = _imu_dr_vel_d;
    ctx.lpos_ground_speed = _lpos_ground_speed;
    ctx.lpos_heading_rad = _lpos_heading_rad;

    ctx.sim_att_valid = _sim_att_valid;
    ctx.sim_att_timestamp = _sim_att_timestamp;
    ctx.sim_roll_rad = _sim_roll_rad;
    ctx.sim_pitch_rad = _sim_pitch_rad;
    ctx.sim_agl_valid = _sim_agl_valid;
    ctx.sim_agl_timestamp = _sim_agl_timestamp;
    ctx.sim_agl_m = _sim_agl_m;

    ctx.ground_distance_valid = _ground_distance_valid;
    ctx.ground_distance_timestamp = _ground_distance_timestamp;
    ctx.ground_distance_m = _ground_distance_m;

    ctx.jmit_baro_valid = _jmit_baro_valid;
    ctx.jmit_baro_timestamp = _jmit_baro_timestamp;
    ctx.jmit_baro_alt_m = _jmit_baro_alt_m;
    ctx.jmit_baro_alt_offset_valid = _jmit_baro_alt_offset_valid;
    ctx.jmit_baro_alt_offset = _jmit_baro_alt_offset;

    ctx.jmit_lpos_alt_valid = _jmit_lpos_alt_valid;
    ctx.jmit_lpos_timestamp = _jmit_lpos_timestamp;
    ctx.jmit_lpos_alt_msl = _jmit_lpos_alt_msl;

    return ctx;
}

// Publica acelerometro/giroscopio, aplica ataque IMU e atualiza estados locais usados na mitigacao GPS.
void GZBridge::imuCallback(const gz::msgs::IMU &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    // Rotacoes fixas usadas para converter orientacao do Gazebo para o PX4.
    static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

    // Conversao da IMU de FLU para FRD antes da publicacao no PX4.
    gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
                         msg.linear_acceleration().x(),
                         msg.linear_acceleration().y(),
                         msg.linear_acceleration().z()));

    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_IMU_DEVTYPE_SIM;
    id.devid_s.bus = 1;
    id.devid_s.address = 1;

    sensor_accel_s accel{};

    accel.timestamp_sample = timestamp;
    accel.timestamp = timestamp;
    accel.device_id = id.devid;

    accel.x = accel_b.X();
    accel.y = accel_b.Y();
    accel.z = accel_b.Z();
    accel.temperature = NAN;
    accel.samples = 1;

    gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
                        msg.angular_velocity().x(),
                        msg.angular_velocity().y(),
                        msg.angular_velocity().z()));

    sensor_gyro_s gyro{};
    gyro.timestamp_sample = timestamp;
    gyro.timestamp = timestamp;
    gyro.device_id = id.devid;
    gyro.x = gyro_b.X();
    gyro.y = gyro_b.Y();
    gyro.z = gyro_b.Z();
    gyro.temperature = NAN;
    gyro.samples = 1;

    // Ataque IMU: pode zerar completamente o sensor ou somar offsets configurados por eixo.
    _imu_attack.apply(accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

    // ======== MITIGACAO IMU - INICIO ========
    // Deteccao e substituicao de anomalia de IMU. As checagens de armado/pouso
    // usam as mesmas subscricoes compartilhadas com outras mitigacoes.
    actuator_armed_s imu_actuator_armed{};
    const bool imu_armed_available = _actuator_armed_sub.copy(&imu_actuator_armed);
    const bool imu_vehicle_armed = imu_armed_available && imu_actuator_armed.armed;

    vehicle_land_detected_s imu_land_detected{};
    const bool imu_land_detector_available = _vehicle_land_detected_sub.copy(&imu_land_detected);
    const bool imu_vehicle_not_landed = imu_land_detector_available && !imu_land_detected.landed;

    _imu_mitigation.detectAndSubstitute(_attack_mitigation_enabled, timestamp,
                        imu_vehicle_armed, imu_vehicle_not_landed,
                        accel.x, accel.y, accel.z,
                        gyro.x, gyro.y, gyro.z);
    // ======== MITIGACAO IMU - FIM ========


    _sensor_accel_pub.publish(accel);
    _sensor_gyro_pub.publish(gyro);

    if (_gps_mitigation.isLandingActive() || _gps_mitigation.isAutoLandSent()) {
        static uint64_t imu_land_log_last_us = 0;

        if ((timestamp - imu_land_log_last_us) > 500000ULL) {
            imu_land_log_last_us = timestamp;

            const bool att_recent =
                _sim_att_valid &&
                (timestamp >= _sim_att_timestamp) &&
                ((timestamp - _sim_att_timestamp) <= 500000ULL);

            const bool agl_recent =
                _sim_agl_valid &&
                (timestamp >= _sim_agl_timestamp) &&
                ((timestamp - _sim_agl_timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US) &&
                PX4_ISFINITE(_sim_agl_m);

            const double roll_deg = att_recent ?
                static_cast<double>(_sim_roll_rad * 57.2957795f) : -999.0;

            const double pitch_deg = att_recent ?
                static_cast<double>(_sim_pitch_rad * 57.2957795f) : -999.0;

            const double agl_m = agl_recent ? _sim_agl_m : -1.0;

            PX4_INFO("\n[Landing-IMU] roll=%.1f pitch=%.1f agl=%.2f | accel=(%.2f %.2f %.2f) | gyro=(%.2f %.2f %.2f) | lpos_v=(%.2f %.2f %.2f)\n",
                roll_deg,
                pitch_deg,
                agl_m,
                static_cast<double>(accel.x),
                static_cast<double>(accel.y),
                static_cast<double>(accel.z),
                static_cast<double>(gyro.x),
                static_cast<double>(gyro.y),
                static_cast<double>(gyro.z),
                static_cast<double>(_imu_dr_vel_n),
                static_cast<double>(_imu_dr_vel_e),
                static_cast<double>(_imu_dr_vel_d));
        }
    }

    // ======== MITIGACAO GPS - INICIO ========
    // Cache da estimativa local usado para validar posicao, calcular velocidade horizontal
    // e manter uma referencia de altitude MSL.

    if (_lpos_ekf2_sub.updated()) {
        vehicle_local_position_s lpos{};
        _lpos_ekf2_sub.copy(&lpos);

        if (lpos.xy_valid && PX4_ISFINITE(lpos.x) && PX4_ISFINITE(lpos.y)) {
            _lpos_n_m = static_cast<double>(lpos.x);
            _lpos_e_m = static_cast<double>(lpos.y);
            _lpos_xy_valid = true;
            _lpos_timestamp = timestamp;
        }

        if (lpos.v_xy_valid) {
            // Atualiza a velocidade horizontal usada no criterio de chegada ao destino.
            _imu_dr_vel_n = math::constrain(lpos.vx, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S);
            _imu_dr_vel_e = math::constrain(lpos.vy, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S);
            _lpos_ground_speed = sqrtf(_imu_dr_vel_n * _imu_dr_vel_n + _imu_dr_vel_e * _imu_dr_vel_e);
        }

        // Captura o heading atual do drone para uso no comando LAND (garante pouso reto).
        if (PX4_ISFINITE(lpos.heading)) {
            _lpos_heading_rad = lpos.heading;
        }
        if (lpos.v_z_valid) {
            _imu_dr_vel_d = math::constrain(lpos.vz, -JMIT_DR_VEL_Z_MAX_M_S, JMIT_DR_VEL_Z_MAX_M_S);
        }

        if (lpos.z_valid && PX4_ISFINITE(lpos.z)) {
            _lpos_alt_msl = static_cast<double>(_alt_ref) - static_cast<double>(lpos.z);
            _lpos_z_valid = true;
            _lpos_timestamp = timestamp;

            _jmit_lpos_alt_msl = _lpos_alt_msl;
            _jmit_lpos_alt_valid = true;
            _jmit_lpos_timestamp = timestamp;
        }
    }

    // ======== MITIGACAO GPS - FIM ========

    // ======== MITIGACAO BLACKOUT GPS - INICIO ========
    GpsMitigationContext gps_ctx = buildGpsMitigationContext();
    sensor_gps_s blackout_pseudo_gps{};

    if (_gps_mitigation.tickBlackout(_attack_mitigation_enabled, timestamp, _pos_ref, gps_ctx,
                     _gps_real_valid, _gps_real_last_us, _gps_real_n_m, _gps_real_e_m, _gps_real_alt,
                     _gps_real_vel_n, _gps_real_vel_e, _sim_gps_used.get(), blackout_pseudo_gps)) {
        _sensor_gps_pub.publish(blackout_pseudo_gps);
    }
    // ======== MITIGACAO BLACKOUT GPS - FIM ========

    // Diagnostico da inclinacao durante a aproximacao e o pouso.
    // Mostra se o roll/pitch aparece por correcao horizontal, velocidade residual ou divergencia entre fontes.
    if (_attack_mitigation_enabled &&
            _gps_mitigation.isBlackoutAnchorInitialized() &&
            _gps_mitigation.isBlackoutDetected() &&
            !_gps_mitigation.isAutoDisarmSent()) {

        static uint64_t tilt_diag_last_us = 0;

        const bool diag_att_recent =
            _sim_att_valid &&
            (timestamp >= _sim_att_timestamp) &&
            ((timestamp - _sim_att_timestamp) <= 500000ULL);

        const bool diag_agl_recent =
            _sim_agl_valid &&
            (timestamp >= _sim_agl_timestamp) &&
            ((timestamp - _sim_agl_timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US) &&
            PX4_ISFINITE(_sim_agl_m);

        const bool diag_vio_recent =
            _vio_valid &&
            (timestamp >= _vio_timestamp) &&
            ((timestamp - _vio_timestamp) <= BLACKOUT_VIO_TIMEOUT_US) &&
            PX4_ISFINITE(_vio_n_m) &&
            PX4_ISFINITE(_vio_e_m) &&
            PX4_ISFINITE(_vio_alt_msl);

        const bool diag_lpos_recent =
            _lpos_xy_valid &&
            _lpos_z_valid &&
            (timestamp >= _lpos_timestamp) &&
            ((timestamp - _lpos_timestamp) <= 500000ULL) &&
            PX4_ISFINITE(_lpos_n_m) &&
            PX4_ISFINITE(_lpos_e_m) &&
            PX4_ISFINITE(_lpos_alt_msl);

        double diag_err_xy_m = -1.0;

        position_setpoint_triplet_s diag_triplet{};

        if (_pos_sp_triplet_sub.copy(&diag_triplet) &&
                diag_triplet.current.valid &&
                PX4_ISFINITE(diag_triplet.current.lat) &&
                PX4_ISFINITE(diag_triplet.current.lon) &&
                _pos_ref.isInitialized()) {

            float diag_dest_n_m = 0.0f;
            float diag_dest_e_m = 0.0f;

            _pos_ref.project(diag_triplet.current.lat, diag_triplet.current.lon,
                diag_dest_n_m, diag_dest_e_m);

            const double diag_dn =
                _gps_mitigation.blackoutDrNm() - static_cast<double>(diag_dest_n_m);

            const double diag_de =
                _gps_mitigation.blackoutDrEm() - static_cast<double>(diag_dest_e_m);

            diag_err_xy_m = sqrt(diag_dn * diag_dn + diag_de * diag_de);
        }

        const bool diag_near_landing =
            _gps_mitigation.isLandingActive() ||
            _gps_mitigation.isAutoLandSent() ||
            (diag_err_xy_m >= 0.0 && diag_err_xy_m <= 3.0) ||
            (diag_agl_recent && _sim_agl_m <= 3.0);

        if (diag_near_landing &&
                ((timestamp - tilt_diag_last_us) > 250000ULL)) {
            tilt_diag_last_us = timestamp;

            const char *diag_phase = _gps_mitigation.isLandingActive() ?
                "LAND_ACTIVE" : (_gps_mitigation.isAutoLandSent() ? "LAND_SENT" : "APPROACH");

            const double diag_roll_deg = diag_att_recent ?
                static_cast<double>(_sim_roll_rad * 57.2957795f) : -999.0;

            const double diag_pitch_deg = diag_att_recent ?
                static_cast<double>(_sim_pitch_rad * 57.2957795f) : -999.0;

            const double diag_agl_m = diag_agl_recent ? _sim_agl_m : -1.0;

            const double diag_lpos_dn =
                diag_lpos_recent ? (_lpos_n_m - _gps_mitigation.blackoutDrNm()) : 999.0;

            const double diag_lpos_de =
                diag_lpos_recent ? (_lpos_e_m - _gps_mitigation.blackoutDrEm()) : 999.0;

            const double diag_vio_dn =
                diag_vio_recent ? (_vio_n_m - _gps_mitigation.blackoutDrNm()) : 999.0;

            const double diag_vio_de =
                diag_vio_recent ? (_vio_e_m - _gps_mitigation.blackoutDrEm()) : 999.0;

            PX4_INFO("\n[Tilt-Diag] phase=%s | roll=%.1f pitch=%.1f agl=%.2f err=%.2f | "
                "lpos_v=(%.2f %.2f %.2f) sp_v=(%.2f %.2f %.2f) | "
                "lpos-pgps=(%.2f %.2f) vio-pgps=(%.2f %.2f) | "
                "pgps=(%.1f %.1f %.2f) lpos=(%.1f %.1f %.2f) vio=(%.1f %.1f %.2f)\n",
                diag_phase,
                diag_roll_deg,
                diag_pitch_deg,
                diag_agl_m,
                diag_err_xy_m,
                static_cast<double>(_imu_dr_vel_n),
                static_cast<double>(_imu_dr_vel_e),
                static_cast<double>(_imu_dr_vel_d),
                static_cast<double>(_gps_mitigation.blackoutDrVelN()),
                static_cast<double>(_gps_mitigation.blackoutDrVelE()),
                static_cast<double>(_gps_mitigation.blackoutDrVelD()),
                diag_lpos_dn,
                diag_lpos_de,
                diag_vio_dn,
                diag_vio_de,
                _gps_mitigation.blackoutDrNm(),
                _gps_mitigation.blackoutDrEm(),
                _gps_mitigation.blackoutDrAlt(),
                diag_lpos_recent ? _lpos_n_m : 999.0,
                diag_lpos_recent ? _lpos_e_m : 999.0,
                diag_lpos_recent ? _lpos_alt_msl : 999.0,
                diag_vio_recent ? _vio_n_m : 999.0,
                diag_vio_recent ? _vio_e_m : 999.0,
                diag_vio_recent ? _vio_alt_msl : 999.0);
        }
    }

    // ======== MITIGACAO DE POUSO GPS - INICIO ========
    _gps_mitigation.checkAutoLand(_attack_mitigation_enabled, timestamp, _pos_ref, _alt_ref, gps_ctx, _gps_real_alt);
    _gps_mitigation.checkAutoDisarm(timestamp, gps_ctx);
    // ======== MITIGACAO DE POUSO GPS - FIM ========

    // ======== MITIGACAO DE POUSO IMU - INICIO ========
    _imu_mitigation.checkAutoLand(timestamp, _pos_ref,
                       _lpos_xy_valid, _lpos_timestamp,
                       _lpos_n_m, _lpos_e_m, _lpos_ground_speed);

    _imu_mitigation.checkAutoDisarm(timestamp,
                       _ground_distance_valid, _ground_distance_timestamp, _ground_distance_m,
                       _sim_agl_valid, _sim_agl_timestamp, _sim_agl_m,
                       _sim_att_valid, _sim_att_timestamp, _sim_roll_rad, _sim_pitch_rad,
                       _lpos_ground_speed);
    // ======== MITIGACAO DE POUSO IMU - FIM ========

    // ======== MITIGACAO DE POUSO MOTOR - INICIO ========
    checkMotorAutoLand(timestamp);
    checkMotorAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO MOTOR - FIM ========

    // ======== MITIGACAO DE POUSO MAGNETOMETRO - INICIO ========
    _mag_mitigation.checkAutoLand(timestamp, _pos_ref,
                       _lpos_xy_valid, _lpos_timestamp,
                       _lpos_n_m, _lpos_e_m, _lpos_ground_speed);

    _mag_mitigation.checkAutoDisarm(timestamp,
                       _ground_distance_valid, _ground_distance_timestamp, _ground_distance_m,
                       _sim_agl_valid, _sim_agl_timestamp, _sim_agl_m,
                       _sim_att_valid, _sim_att_timestamp, _sim_roll_rad, _sim_pitch_rad,
                       _lpos_ground_speed);
    // ======== MITIGACAO DE POUSO MAGNETOMETRO - FIM ========

    // ======== MITIGACAO DE POUSO LIDAR - INICIO ========
    _lidar_mitigation.checkAutoLandDown(timestamp, _pos_ref,
                       _lpos_xy_valid, _lpos_timestamp,
                       _lpos_n_m, _lpos_e_m, _lpos_ground_speed);

    _lidar_mitigation.checkAutoDisarmDown(timestamp,
                       _ground_distance_valid, _ground_distance_timestamp, _ground_distance_m,
                       _sim_agl_valid, _sim_agl_timestamp, _sim_agl_m,
                       _sim_att_valid, _sim_att_timestamp, _sim_roll_rad, _sim_pitch_rad,
                       _lpos_ground_speed);
    // ======== MITIGACAO DE POUSO LIDAR - FIM ========

    // ======== MITIGACAO DE POUSO LIDAR FRONTAL - INICIO ========
    _lidar_mitigation.checkAutoLandFront(timestamp, _pos_ref,
                       _lpos_xy_valid, _lpos_timestamp,
                       _lpos_n_m, _lpos_e_m, _lpos_ground_speed);

    _lidar_mitigation.checkAutoDisarmFront(timestamp,
                       _ground_distance_valid, _ground_distance_timestamp, _ground_distance_m,
                       _sim_agl_valid, _sim_agl_timestamp, _sim_agl_m,
                       _sim_att_valid, _sim_att_timestamp, _sim_roll_rad, _sim_pitch_rad,
                       _lpos_ground_speed);
    // ======== MITIGACAO DE POUSO LIDAR FRONTAL - FIM ========

    // ======== MITIGACAO DE POUSO LIDAR 2D - INICIO ========
    _lidar_mitigation.checkAutoLandTwoD(timestamp, _pos_ref,
                       _lpos_xy_valid, _lpos_timestamp,
                       _lpos_n_m, _lpos_e_m, _lpos_ground_speed);

    _lidar_mitigation.checkAutoDisarmTwoD(timestamp,
                       _ground_distance_valid, _ground_distance_timestamp, _ground_distance_m,
                       _sim_agl_valid, _sim_agl_timestamp, _sim_agl_m,
                       _sim_att_valid, _sim_att_timestamp, _sim_roll_rad, _sim_pitch_rad,
                       _lpos_ground_speed);
    // ======== MITIGACAO DE POUSO LIDAR 2D - FIM ========

    // ======== MITIGACAO DE POUSO BAROMETRO - INICIO ========
    checkBaroAutoLand(timestamp);
    checkBaroAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO BAROMETRO - FIM ========

}


// ======== MITIGACAO DE POUSO GENERICA (IMU/MOTOR/MAGNETOMETRO) - INICIO ========
// Molde comum de pouso/desarme reaproveitado por IMU, motor e magnetometro:
// cada uma dessas mitigacoes tem uma unica fonte de anomalia e uma unica
// fonte de posicao confiavel (_lpos_*). GPS mantem implementacao propria por
// ter duas origens distintas (blackout e anomalia por ruido).
void GZBridge::checkGenericAnomalyAutoLand(
    uint64_t timestamp,
    bool anomaly_active,
    bool &landing_active,
    bool &auto_land_sent,
    uint64_t &auto_land_arrival_us,
    uint64_t &auto_land_last_arrived_us,
    double &land_hold_n_m,
    double &land_hold_e_m,
    const char *log_tag)
{
    if (!_attack_mitigation_enabled || !anomaly_active ||
            auto_land_sent || !_pos_ref.isInitialized()) {
        auto_land_arrival_us = 0;
        auto_land_last_arrived_us = 0;
        return;
    }

    if (!_lpos_xy_valid || (timestamp - _lpos_timestamp) > 500000ULL) {
        auto_land_arrival_us = 0;
        auto_land_last_arrived_us = 0;
        return;
    }

    position_setpoint_triplet_s triplet{};

    if (!_pos_sp_triplet_sub.copy(&triplet) || !triplet.current.valid ||
            !PX4_ISFINITE(triplet.current.lat) || !PX4_ISFINITE(triplet.current.lon)) {
        auto_land_arrival_us = 0;
        auto_land_last_arrived_us = 0;
        return;
    }

    float dest_n_m = 0.0f;
    float dest_e_m = 0.0f;
    _pos_ref.project(triplet.current.lat, triplet.current.lon, dest_n_m, dest_e_m);

    const double dn = _lpos_n_m - static_cast<double>(dest_n_m);
    const double de = _lpos_e_m - static_cast<double>(dest_e_m);
    const float horizontal_error = static_cast<float>(sqrt(dn * dn + de * de));

    const bool arrived =
        (horizontal_error <= AUTO_LAND_ANOMALY_DEST_RADIUS_M) &&
        (_lpos_ground_speed <= AUTO_LAND_ANOMALY_MAX_SPEED_M_S);

    if (!arrived) {
        auto_land_arrival_us = 0;
        auto_land_last_arrived_us = 0;
        return;
    }

    auto_land_last_arrived_us = timestamp;

    if (auto_land_arrival_us == 0) {
        auto_land_arrival_us = timestamp;

        land_hold_n_m = _lpos_n_m;
        land_hold_e_m = _lpos_e_m;

        PX4_INFO("\n[%s-Land] Destination reached | err_xy=%.1f m | vel=%.2f m/s | window=%.1f s\n",
            log_tag,
            static_cast<double>(horizontal_error),
            static_cast<double>(_lpos_ground_speed),
            static_cast<double>(AUTO_LAND_STABLE_US) / 1e6);

        return;
    }

    const uint64_t preland_elapsed_us = timestamp - auto_land_arrival_us;

    if (preland_elapsed_us < AUTO_LAND_STABLE_US) {
        return;
    }

    landing_active = true;
    auto_land_sent = true;

    publishGenericLandCommand(timestamp, land_hold_n_m, land_hold_e_m, log_tag);
}

// Envia comando de desarme assim que o detector de pouso do PX4 (ou, em ultimo
// caso, os sensores de ground-truth do Gazebo usados apenas para confirmar o
// solo) indicar contato com o solo. A anomalia original nunca e usada como
// criterio aqui.
void GZBridge::checkGenericAnomalyAutoDisarm(
    uint64_t timestamp,
    bool &landing_active,
    bool &auto_land_sent,
    bool &auto_disarm_sent,
    uint32_t &sim_agl_ground_count,
    const char *log_tag)
{
    if (!landing_active || !auto_land_sent || auto_disarm_sent) {
        return;
    }

    vehicle_land_detected_s land_detected{};
    const bool land_detector_available = _vehicle_land_detected_sub.copy(&land_detected);

    const bool land_detector_recent =
        land_detector_available &&
        (land_detected.timestamp > 0) &&
        (timestamp >= land_detected.timestamp) &&
        ((timestamp - land_detected.timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US);

    const bool ground_contact_by_px4_landed =
        land_detector_recent && land_detected.landed;

    const bool ground_contact_by_px4_early =
        land_detector_recent &&
        (land_detected.ground_contact || land_detected.maybe_landed);

    const bool ground_sensor_recent =
        _ground_distance_valid &&
        ((timestamp - _ground_distance_timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US);

    const bool ground_contact_by_sensor =
        ground_sensor_recent &&
        PX4_ISFINITE(_ground_distance_m) &&
        (_ground_distance_m <= AUTO_LAND_GROUND_DISARM_DIST_M);

    // Ground-truth do Gazebo, usado exclusivamente para confirmar contato com
    // o solo durante o pouso, nunca para detectar a anomalia original.
    const bool sim_ground_recent =
        _sim_agl_valid &&
        (timestamp >= _sim_agl_timestamp) &&
        ((timestamp - _sim_agl_timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US) &&
        PX4_ISFINITE(_sim_agl_m);

    if (sim_ground_recent) {
        if (_sim_agl_m <= AUTO_LAND_SIM_GROUND_DISARM_AGL_M) {
            sim_agl_ground_count++;
        } else {
            sim_agl_ground_count = 0;
        }
    } else {
        sim_agl_ground_count = 0;
    }

    const bool ground_contact_by_sim_agl =
        sim_ground_recent &&
        (_sim_agl_m <= AUTO_LAND_SIM_GROUND_DISARM_AGL_M) &&
        (sim_agl_ground_count >= SIM_AGL_GROUND_CONFIRM_COUNT);

    if (!ground_contact_by_px4_landed &&
            !ground_contact_by_px4_early &&
            !ground_contact_by_sensor &&
            !ground_contact_by_sim_agl) {
        return;
    }

    const bool force_disarm = !ground_contact_by_px4_landed;

    const bool disarm_att_recent =
        _sim_att_valid &&
        (timestamp >= _sim_att_timestamp) &&
        ((timestamp - _sim_att_timestamp) <= 500000ULL);

    const bool forced_disarm_speed_ok = (_lpos_ground_speed <= 0.15f);

    const bool forced_disarm_att_ok =
        !disarm_att_recent ||
        ((fabsf(_sim_roll_rad) <= AUTO_LAND_MAX_ROLL_RAD) &&
        (fabsf(_sim_pitch_rad) <= AUTO_LAND_MAX_PITCH_RAD));

    if (force_disarm && (!forced_disarm_speed_ok || !forced_disarm_att_ok)) {
        return;
    }

    if (ground_contact_by_px4_landed) {
        PX4_WARN("\n[%s-Land] PX4 landing detector confirmed landed - normal disarm\n", log_tag);

    } else if (ground_contact_by_px4_early) {
        PX4_WARN("\n[%s-Land] PX4 landing detector reported ground contact - safety disarm\n", log_tag);

    } else if (ground_contact_by_sensor) {
        PX4_WARN("\n[%s-Land] Downward distance sensor reported ground contact | dist=%.2f m - safety disarm\n",
            log_tag,
            static_cast<double>(_ground_distance_m));

    } else {
        PX4_WARN("\n[%s-Land] Simulated ground proximity reached | agl=%.2f m - safety disarm\n",
            log_tag,
            static_cast<double>(_sim_agl_m));
    }

    publishGenericDisarmCommand(timestamp, force_disarm, log_tag);
    auto_disarm_sent = true;
    auto_land_sent = true;
    landing_active = false;
}

// Envia LAND com coordenada global explicita no ponto de chegada (posicao
// fixa; a descida vertical fica sob controle do modo LAND do PX4).
void GZBridge::publishGenericLandCommand(uint64_t timestamp, double land_hold_n_m, double land_hold_e_m, const char *log_tag)
{
    double land_lat = 0.0;
    double land_lon = 0.0;
    _pos_ref.reproject(
        static_cast<float>(land_hold_n_m),
        static_cast<float>(land_hold_e_m),
        land_lat,
        land_lon);

    vehicle_command_s cmd{};
    cmd.timestamp = timestamp;
    cmd.param1 = 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = NAN;
    cmd.param5 = static_cast<float>(land_lat);
    cmd.param6 = static_cast<float>(land_lon);
    cmd.param7 = NAN;

    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 0;
    cmd.from_external = false;

    _vehicle_command_pub.publish(cmd);

    PX4_WARN("\n[%s-Land] Controlled landing started | hold N=%.1f E=%.1f | lat=%.7f lon=%.7f\n",
        log_tag,
        land_hold_n_m,
        land_hold_e_m,
        land_lat,
        land_lon);
}

// Envia o comando de desarme do PX4 apos confirmacao de contato com o solo.
void GZBridge::publishGenericDisarmCommand(uint64_t timestamp, bool force_disarm, const char *log_tag)
{
    vehicle_command_s cmd{};
    cmd.timestamp = timestamp;
    cmd.param1 = 0.0f;
    cmd.param2 = force_disarm ? 21196.0f : 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 0;
    cmd.from_external = false;

    _vehicle_command_pub.publish(cmd);

    PX4_WARN("\n[%s-Land] %s disarm command sent after ground contact confirmation.\n",
        log_tag,
        force_disarm ? "Forced" : "Normal");
}
// ======== MITIGACAO DE POUSO GENERICA - FIM ========

// ======== MITIGACAO DE POUSO IMU - INICIO ========
// Pouso proprio de IMU: ver ImuMitigation::checkAutoLand/checkAutoDisarm.
// void GZBridge::checkImuAutoLand(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoLand(timestamp, _imu_mitigation.isAnomalyActive(), _imu_landing_active,
        // _imu_auto_land_sent, _imu_auto_land_arrival_us, _imu_auto_land_last_arrived_us,
        // _imu_land_hold_n_m, _imu_land_hold_e_m, "IMU");
// }

// void GZBridge::checkImuAutoDisarm(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoDisarm(timestamp, _imu_landing_active, _imu_auto_land_sent,
        // _imu_auto_disarm_sent, _imu_sim_agl_ground_count, "IMU");
// }
// ======== MITIGACAO DE POUSO IMU - FIM ========

// ======== MITIGACAO DE POUSO MOTOR - INICIO ========
void GZBridge::checkMotorAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _mixing_interface_esc.motorAnomalyActive(), _motor_landing_active,
        _motor_auto_land_sent, _motor_auto_land_arrival_us, _motor_auto_land_last_arrived_us,
        _motor_land_hold_n_m, _motor_land_hold_e_m, "Motor");
}

void GZBridge::checkMotorAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _motor_landing_active, _motor_auto_land_sent,
        _motor_auto_disarm_sent, _motor_sim_agl_ground_count, "Motor");
}
// ======== MITIGACAO DE POUSO MOTOR - FIM ========

// ======== MITIGACAO DE POUSO MAGNETOMETRO - INICIO ========
// Pouso proprio de magnetometro: ver MagnetometerMitigation::checkAutoLand/checkAutoDisarm.
// void GZBridge::checkMagAutoLand(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoLand(timestamp, _mag_anomaly_active, _mag_landing_active,
        // _mag_auto_land_sent, _mag_auto_land_arrival_us, _mag_auto_land_last_arrived_us,
        // _mag_land_hold_n_m, _mag_land_hold_e_m, "Mag");
// }

// void GZBridge::checkMagAutoDisarm(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoDisarm(timestamp, _mag_landing_active, _mag_auto_land_sent,
        // _mag_auto_disarm_sent, _mag_sim_agl_ground_count, "Mag");
// }
// ======== MITIGACAO DE POUSO MAGNETOMETRO - FIM ========

// ======== MITIGACAO DE POUSO LIDAR - INICIO ========
// Pouso proprio dos tres sensores LiDAR: ver LidarMitigation::checkAutoLandDown/
// Front/TwoD e checkAutoDisarmDown/Front/TwoD.
// void GZBridge::checkLidarAutoLand(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoLand(timestamp, _lidar_anomaly_active, _lidar_landing_active,
        // _lidar_auto_land_sent, _lidar_auto_land_arrival_us, _lidar_auto_land_last_arrived_us,
        // _lidar_land_hold_n_m, _lidar_land_hold_e_m, "Lidar");
// }

// void GZBridge::checkLidarAutoDisarm(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoDisarm(timestamp, _lidar_landing_active, _lidar_auto_land_sent,
        // _lidar_auto_disarm_sent, _lidar_sim_agl_ground_count, "Lidar");
// }
// ======== MITIGACAO DE POUSO LIDAR - FIM ========

// ======== MITIGACAO DE POUSO LIDAR FRONTAL - INICIO ========
// void GZBridge::checkLidarFrontAutoLand(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoLand(timestamp, _lidar_front_anomaly_active, _lidar_front_landing_active,
        // _lidar_front_auto_land_sent, _lidar_front_auto_land_arrival_us, _lidar_front_auto_land_last_arrived_us,
        // _lidar_front_land_hold_n_m, _lidar_front_land_hold_e_m, "LidarFront");
// }

// void GZBridge::checkLidarFrontAutoDisarm(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoDisarm(timestamp, _lidar_front_landing_active, _lidar_front_auto_land_sent,
        // _lidar_front_auto_disarm_sent, _lidar_front_sim_agl_ground_count, "LidarFront");
// }
// ======== MITIGACAO DE POUSO LIDAR FRONTAL - FIM ========

// ======== MITIGACAO DE POUSO LIDAR 2D - INICIO ========
// void GZBridge::checkLidar2dAutoLand(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoLand(timestamp, _lidar2d_anomaly_active, _lidar2d_landing_active,
        // _lidar2d_auto_land_sent, _lidar2d_auto_land_arrival_us, _lidar2d_auto_land_last_arrived_us,
        // _lidar2d_land_hold_n_m, _lidar2d_land_hold_e_m, "Lidar2D");
// }

// void GZBridge::checkLidar2dAutoDisarm(uint64_t timestamp)
// {
    // checkGenericAnomalyAutoDisarm(timestamp, _lidar2d_landing_active, _lidar2d_auto_land_sent,
        // _lidar2d_auto_disarm_sent, _lidar2d_sim_agl_ground_count, "Lidar2D");
// }
// ======== MITIGACAO DE POUSO LIDAR 2D - FIM ========

// ======== MITIGACAO DE POUSO BAROMETRO - INICIO ========
void GZBridge::checkBaroAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _baro_anomaly_active, _baro_landing_active,
        _baro_auto_land_sent, _baro_auto_land_arrival_us, _baro_auto_land_last_arrived_us,
        _baro_land_hold_n_m, _baro_land_hold_e_m, "Baro");
}

void GZBridge::checkBaroAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _baro_landing_active, _baro_auto_land_sent,
        _baro_auto_disarm_sent, _baro_sim_agl_ground_count, "Baro");
}
// ======== MITIGACAO DE POUSO BAROMETRO - FIM ========



// Publica estados simulados auxiliares de atitude, velocidade angular e posicao local.
void GZBridge::poseInfoCallback(const gz::msgs::Pose_V &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    // Procura a pose correspondente ao modelo controlado pelo PX4.
    for (int p = 0; p < msg.pose_size(); p++) {
        if (msg.pose(p).name() == _model_name) {

            // Intervalo de predicao limitado para evitar saltos em callbacks atrasadas.
            const double dt = math::constrain((timestamp - _timestamp_prev) * 1e-6, 0.001, 0.1);
            _timestamp_prev = timestamp;

            gz::msgs::Vector3d pose_position = msg.pose(p).position();
            gz::msgs::Quaternion pose_orientation = msg.pose(p).orientation();

            _sim_agl_m = pose_position.z();
            _sim_agl_valid = PX4_ISFINITE(_sim_agl_m);
            _sim_agl_timestamp = timestamp;

            gz::math::Quaterniond q_gr = gz::math::Quaterniond(
                                 pose_orientation.w(),
                                 pose_orientation.x(),
                                 pose_orientation.y(),
                                 pose_orientation.z());

            gz::math::Quaterniond q_nb;
            GZBridge::rotateQuaternion(q_nb, q_gr);

            vehicle_attitude_s vehicle_attitude_groundtruth{};
            vehicle_attitude_groundtruth.timestamp_sample = timestamp;
            vehicle_attitude_groundtruth.q[0] = q_nb.W();
            vehicle_attitude_groundtruth.q[1] = q_nb.X();
            vehicle_attitude_groundtruth.q[2] = q_nb.Y();
            vehicle_attitude_groundtruth.q[3] = q_nb.Z();
            vehicle_attitude_groundtruth.timestamp = timestamp;
            _attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

            // Quaternion NED<-body atual.
            const matrix::Quatf q_nb_now(vehicle_attitude_groundtruth.q);

            // Velocidade angular via diferenca de quaternions, sem wrap-around de Euler.
            // q_delta = q_prev^{-1} * q_now  =>  v_body = 2 * vec(q_delta) / dt
            // Isso evita o spike que ocorre quando yaw cruza +/-pi durante o pouso,
            // que causava 'Attitude failure (roll)' no EKF do PX4.
            vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
            vehicle_angular_velocity_groundtruth.timestamp_sample = timestamp;
            {
                const matrix::Quatf q_delta = _q_nb_prev.inversed() * q_nb_now;
                // Garante sinal consistente (caminho mais curto no espaco de quaternions).
                const float w = q_delta(0) >= 0.0f ? q_delta(0) : -q_delta(0);
                const float sign = q_delta(0) >= 0.0f ? 1.0f : -1.0f;
                const matrix::Vector3f omega_body{
                    sign * q_delta(1),
                    sign * q_delta(2),
                    sign * q_delta(3)
                };
                // omega = 2 * vec(q_delta) / dt  (aproximacao valida para dt pequeno)
                const float dt_f = static_cast<float>(dt);
                const matrix::Vector3f angular_velocity = (2.0f / dt_f) * omega_body;
                (void)w;  // w usada apenas para determinar o sinal
                angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);
            }
            _q_nb_prev = q_nb_now;

            vehicle_angular_velocity_groundtruth.timestamp = timestamp;
            _angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

            vehicle_local_position_s local_position_groundtruth{};
            local_position_groundtruth.timestamp_sample = timestamp;

            // Conversao de posicao ENU do Gazebo para NED do PX4.
            const matrix::Vector3d position{pose_position.y(), pose_position.x(), -pose_position.z()};
            const matrix::Vector3d velocity{(position - _position_prev) / dt};
            const matrix::Vector3d acceleration{(velocity - _velocity_prev) / dt};

            _position_prev = position;
            _velocity_prev = velocity;

            local_position_groundtruth.ax = acceleration(0);
            local_position_groundtruth.ay = acceleration(1);
            local_position_groundtruth.az = acceleration(2);
            local_position_groundtruth.vx = velocity(0);
            local_position_groundtruth.vy = velocity(1);
            local_position_groundtruth.vz = velocity(2);
            local_position_groundtruth.x = position(0);
            local_position_groundtruth.y = position(1);
            local_position_groundtruth.z = position(2);

            // heading via quaternion: atan2 da parte yaw para evitar gimbal lock.
            const matrix::Eulerf euler_now{q_nb_now};
            local_position_groundtruth.heading = euler_now.psi();

            _sim_roll_rad = euler_now.phi();
            _sim_pitch_rad = euler_now.theta();
            _sim_att_valid = PX4_ISFINITE(_sim_roll_rad) && PX4_ISFINITE(_sim_pitch_rad);
            _sim_att_timestamp = timestamp;

            if (_pos_ref.isInitialized()) {

                local_position_groundtruth.ref_lat = _pos_ref.getProjectionReferenceLat();
                local_position_groundtruth.ref_lon = _pos_ref.getProjectionReferenceLon();
                local_position_groundtruth.ref_alt = _alt_ref;
                local_position_groundtruth.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
                local_position_groundtruth.xy_global = true;
                local_position_groundtruth.z_global = true;

            } else {
                local_position_groundtruth.ref_lat = static_cast<double>(NAN);
                local_position_groundtruth.ref_lon = static_cast<double>(NAN);
                local_position_groundtruth.ref_alt = NAN;
                local_position_groundtruth.ref_timestamp = 0;
                local_position_groundtruth.xy_global = false;
                local_position_groundtruth.z_global = false;
            }

            local_position_groundtruth.timestamp = timestamp;
            _lpos_ground_truth_pub.publish(local_position_groundtruth);
            return;
        }
    }
}

// Converte a odometria do Gazebo para o referencial PX4 e publica como odometria visual.
void GZBridge::odometryCallback(const gz::msgs::OdometryWithCovariance &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    vehicle_odometry_s report{};
    report.timestamp_sample = timestamp;
    report.timestamp = timestamp;

    // Posicao ENU do Gazebo convertida para NED.
    report.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
    report.position[0] = msg.pose_with_covariance().pose().position().y();
    report.position[1] = msg.pose_with_covariance().pose().position().x();
    report.position[2] = -msg.pose_with_covariance().pose().position().z();

    // Variaveis usadas mais abaixo (apos o quaternion ser calculado) para
    // reconstruir o substituto dinamico de IMU a partir da VIO.
    bool   vio_sample_accepted = false;
    double vio_accel_n_ned = 0.0;
    double vio_accel_e_ned = 0.0;
    double vio_accel_d_ned = 0.0;
    bool   vio_accel_ned_valid = false;

    // ======== MITIGACAO BLACKOUT GPS - INICIO ========
    // Cache VIO used as auxiliary localization during GPS blackout.
    // The cache is accepted only when covariance and motion are plausible.

    if (_pos_ref.isInitialized() &&
            PX4_ISFINITE(report.position[0]) &&
            PX4_ISFINITE(report.position[1]) &&
            PX4_ISFINITE(report.position[2])) {

        const double vio_n = static_cast<double>(report.position[0]);
        const double vio_e = static_cast<double>(report.position[1]);
        const double vio_d = static_cast<double>(report.position[2]);

        const float vio_var_n = static_cast<float>(msg.pose_with_covariance().covariance().data(7));
        const float vio_var_e = static_cast<float>(msg.pose_with_covariance().covariance().data(0));
        const float vio_var_d = static_cast<float>(msg.pose_with_covariance().covariance().data(14));

        const bool vio_cov_valid =
            PX4_ISFINITE(vio_var_n) && PX4_ISFINITE(vio_var_e) && PX4_ISFINITE(vio_var_d) &&
            vio_var_n >= 0.0f && vio_var_e >= 0.0f && vio_var_d >= 0.0f &&
            vio_var_n <= VIO_MAX_POS_VAR_M2 &&
            vio_var_e <= VIO_MAX_POS_VAR_M2 &&
            vio_var_d <= VIO_MAX_POS_VAR_M2;

        float vio_vel_n = _vio_vel_n;
        float vio_vel_e = _vio_vel_e;
        float vio_vel_d = _vio_vel_d;
        bool vio_motion_valid = true;

        if (_vio_valid && _vio_timestamp > 0 && timestamp > _vio_timestamp) {
            const double dt = math::constrain(
                static_cast<double>(timestamp - _vio_timestamp) * 1e-6, 0.001, 0.2);

            vio_vel_n = static_cast<float>((vio_n - _vio_n_m) / dt);
            vio_vel_e = static_cast<float>((vio_e - _vio_e_m) / dt);
            vio_vel_d = static_cast<float>((vio_d - _vio_d_m) / dt);

            // ======== MITIGACAO IMU - INICIO ========
            // Aceleracao NED por diferenca finita da velocidade VIO (valor
            // anterior de _vio_vel_n/e/d contra a nova estimativa, mesmo dt).
            vio_accel_n_ned = (static_cast<double>(vio_vel_n) - static_cast<double>(_vio_vel_n)) / dt;
            vio_accel_e_ned = (static_cast<double>(vio_vel_e) - static_cast<double>(_vio_vel_e)) / dt;
            vio_accel_d_ned = (static_cast<double>(vio_vel_d) - static_cast<double>(_vio_vel_d)) / dt;
            vio_accel_ned_valid =
                PX4_ISFINITE(vio_accel_n_ned) &&
                PX4_ISFINITE(vio_accel_e_ned) &&
                PX4_ISFINITE(vio_accel_d_ned);
            // ======== MITIGACAO IMU - FIM ========

            const float vio_speed_xy = sqrtf(vio_vel_n * vio_vel_n + vio_vel_e * vio_vel_e);
            vio_motion_valid =
                PX4_ISFINITE(vio_speed_xy) &&
                PX4_ISFINITE(vio_vel_d) &&
                vio_speed_xy <= JMIT_DR_VEL_MAX_M_S &&
                fabsf(vio_vel_d) <= JMIT_DR_VEL_Z_MAX_M_S;
        }

        if (vio_cov_valid && vio_motion_valid) {
            _vio_n_m = vio_n;
            _vio_e_m = vio_e;
            _vio_d_m = vio_d;
            _vio_alt_msl = static_cast<double>(_alt_ref) - vio_d;
            _vio_vel_n = math::constrain(vio_vel_n, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S);
            _vio_vel_e = math::constrain(vio_vel_e, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S);
            _vio_vel_d = math::constrain(vio_vel_d, -JMIT_DR_VEL_Z_MAX_M_S, JMIT_DR_VEL_Z_MAX_M_S);
            _vio_timestamp = timestamp;
            _vio_valid = true;
            vio_sample_accepted = true;
        } else {
            _vio_valid = false;
        }
    }
    // ======== MITIGACAO BLACKOUT GPS - FIM ========

    gz::msgs::Quaternion pose_orientation = msg.pose_with_covariance().pose().orientation();
    gz::math::Quaterniond q_gr = gz::math::Quaterniond(
                         pose_orientation.w(),
                         pose_orientation.x(),
                         pose_orientation.y(),
                         pose_orientation.z());
    gz::math::Quaterniond q_nb;
    GZBridge::rotateQuaternion(q_nb, q_gr);
    report.q[0] = q_nb.W();
    report.q[1] = q_nb.X();
    report.q[2] = q_nb.Y();
    report.q[3] = q_nb.Z();

    // ======== MITIGACAO IMU - INICIO ========
    // Reconstroi accel/gyro a partir da VIO (odometria visual), recalculado a
    // cada atualizacao. Consumido por ImuMitigation::detectAndSubstitute()
    // quando uma anomalia esta ativa.
    if (vio_sample_accepted) {

        // Aceleracao: aceleracao NED (por diferenca da velocidade VIO) menos a
        // gravidade local, rotacionada para o corpo usando a atitude da propria VIO.
        if (vio_accel_ned_valid) {
            const matrix::Quatf q_nb_matrix(q_nb.W(), q_nb.X(), q_nb.Y(), q_nb.Z());
            const matrix::Dcmf dcm_bn(q_nb_matrix); // corpo -> NED

            const matrix::Vector3f specific_force_ned(
                static_cast<float>(vio_accel_n_ned),
                static_cast<float>(vio_accel_e_ned),
                static_cast<float>(vio_accel_d_ned - 9.80665));

            const matrix::Vector3f specific_force_body = dcm_bn.transpose() * specific_force_ned;

            if (PX4_ISFINITE(specific_force_body(0)) &&
                    PX4_ISFINITE(specific_force_body(1)) &&
                    PX4_ISFINITE(specific_force_body(2))) {
                _imu_mitigation.updateVioAccelSubstitute(timestamp,
                                     specific_force_body(0),
                                     specific_force_body(1),
                                     specific_force_body(2));
            }
        }

        // Giroscopio: a velocidade angular do proprio corpo ja vem pronta na
        // mensagem de odometria (twist.angular), sem necessidade de diferenciar
        // quaternion. Mesma convencao de sinal usada para a velocidade linear
        // (FLU -> FRD: X mantido, Y e Z invertidos).
        const float vio_gyro_x = static_cast<float>(msg.twist_with_covariance().twist().angular().x());
        const float vio_gyro_y = static_cast<float>(-msg.twist_with_covariance().twist().angular().y());
        const float vio_gyro_z = static_cast<float>(-msg.twist_with_covariance().twist().angular().z());

        if (PX4_ISFINITE(vio_gyro_x) && PX4_ISFINITE(vio_gyro_y) && PX4_ISFINITE(vio_gyro_z)) {
            _imu_mitigation.updateVioGyroSubstitute(timestamp, vio_gyro_x, vio_gyro_y, vio_gyro_z);
        }
    }
    // ======== MITIGACAO IMU - FIM ========

    // ======== MITIGACAO MAGNETOMETRO - INICIO ========
    // Extrai heading, roll e pitch da atitude da propria VIO, referencia
    // independente do magnetometro e do LiDAR frontal/2D. Cada mitigacao
    // mantem sua propria copia (nao mais um cache compartilhado no GZBridge).
    if (vio_sample_accepted) {
        const matrix::Quatf q_nb_att(q_nb.W(), q_nb.X(), q_nb.Y(), q_nb.Z());
        const matrix::Eulerf euler_vio{q_nb_att};

        if (PX4_ISFINITE(euler_vio.psi()) && PX4_ISFINITE(euler_vio.phi()) && PX4_ISFINITE(euler_vio.theta())) {
            _mag_mitigation.updateVioAttitude(timestamp, euler_vio.psi(), euler_vio.phi(), euler_vio.theta());
            _lidar_mitigation.updateVioAttitude(timestamp, euler_vio.psi(), euler_vio.phi(), euler_vio.theta());
        }
    }
    // ======== MITIGACAO MAGNETOMETRO - FIM ========

    // Velocidade linear convertida de FLU para FRD.
    report.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
    report.velocity[0] = msg.twist_with_covariance().twist().linear().x();
    report.velocity[1] = -msg.twist_with_covariance().twist().linear().y();
    report.velocity[2] = -msg.twist_with_covariance().twist().linear().z();

    report.angular_velocity[0] = msg.twist_with_covariance().twist().angular().x();
    report.angular_velocity[1] = -msg.twist_with_covariance().twist().angular().y();
    report.angular_velocity[2] = -msg.twist_with_covariance().twist().angular().z();

    report.position_variance[0] = msg.pose_with_covariance().covariance().data(7);
    report.position_variance[1] = msg.pose_with_covariance().covariance().data(0);
    report.position_variance[2] = msg.pose_with_covariance().covariance().data(14);

    report.orientation_variance[0] = msg.pose_with_covariance().covariance().data(21);
    report.orientation_variance[1] = msg.pose_with_covariance().covariance().data(28);
    report.orientation_variance[2] = msg.pose_with_covariance().covariance().data(35);

    report.velocity_variance[0] = msg.twist_with_covariance().covariance().data(7);
    report.velocity_variance[1] = msg.twist_with_covariance().covariance().data(0);
    report.velocity_variance[2] = msg.twist_with_covariance().covariance().data(14);

    _visual_odometry_pub.publish(report);
}

// Gera uma amostra de ruido branco gaussiano com media zero e desvio padrao unitario.
float GZBridge::generate_wgn()
{

    // Implementacao Box-Muller polar com reaproveitamento alternado das amostras.
    static float V1, V2, S;
    static bool phase = true;
    float X;

    if (phase) {
        do {
            float U1 = (float)rand() / (float)RAND_MAX;
            float U2 = (float)rand() / (float)RAND_MAX;
            V1 = 2.0f * U1 - 1.0f;
            V2 = 2.0f * U2 - 1.0f;
            S = V1 * V1 + V2 * V2;
        } while (S >= 1.0f || fabsf(S) < 1e-8f);

        X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

    } else {
        X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
    }

    phase = !phase;
    return X;
}

// Aplica o modelo nominal de ruido GPS do simulador a posicao e velocidade.
void GZBridge::addGpsNoise(double &latitude, double &longitude, double &altitude,
               float &vel_north, float &vel_east, float &vel_down)
{
    // Ruido correlacionado de posicao GPS nos eixos Norte, Leste e vertical.
    _gps_pos_noise_n = _pos_markov_time * _gps_pos_noise_n +
               _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
               0.02f * _gps_pos_noise_n;

    _gps_pos_noise_e = _pos_markov_time * _gps_pos_noise_e +
               _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
               0.02f * _gps_pos_noise_e;

    _gps_pos_noise_d = _pos_markov_time * _gps_pos_noise_d +
               _pos_random_walk * generate_wgn() * _pos_noise_amplitude * 1.5f -
               0.02f * _gps_pos_noise_d;

    latitude += math::degrees((double)_gps_pos_noise_n / CONSTANTS_RADIUS_OF_EARTH);
    longitude += math::degrees((double)_gps_pos_noise_e / CONSTANTS_RADIUS_OF_EARTH);
    altitude += (double)_gps_pos_noise_d;

    // Ruido correlacionado de velocidade GPS nos eixos NED.
    _gps_vel_noise_n = _vel_markov_time * _gps_vel_noise_n +
               _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

    _gps_vel_noise_e = _vel_markov_time * _gps_vel_noise_e +
               _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

    _gps_vel_noise_d = _vel_markov_time * _gps_vel_noise_d +
               _vel_noise_density * generate_wgn() * _vel_noise_amplitude * 1.2f;

    vel_north += _gps_vel_noise_n;
    vel_east += _gps_vel_noise_e;
    vel_down += _gps_vel_noise_d;
}

// Processa a amostra GPS: aplica ataques, executa a mitigacao e publica sensor_gps no PX4.
void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    // Ataque 2: blackout GPS. O ataque apenas bloqueia a publicacao do GPS real.
    // A mitigacao detecta a falha por timeout
    // da ultima publicacao GPS real no imuCallback().
    if (_jamming_attack.isBlackoutActive()) {

        // Garante que a origem de projecao local exista para permitir reprojecao do pseudo-GPS.
        if (!_pos_ref.isInitialized()) {
            _pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
            _alt_ref = msg.altitude();
        }

        return;
    }

    // Apos o desarme, interrompe a publicacao de medicoes GPS no chao.
    if (_gps_mitigation.isAutoDisarmSent()) {
        return;
    }

    // A primeira amostra define a origem local usada para projecao/reprojecao GPS.
    if (!_pos_ref.isInitialized()) {
        _pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
        _alt_ref = msg.altitude();
        return;
    }

    // Ataques GPS por offset e rotacao sao aplicados antes do ruido nominal.
    double lat_offset = _gps_offset_attack.offsetLatDeg();
    double lon_offset = _gps_offset_attack.offsetLonDeg();

    if (fabs(_gps_rot_attack.radiusDeg()) > 1e-6 || fabs(_gps_rot_attack.angleDeg()) > 1e-6) {
        double angle_rad = math::radians(_gps_rot_attack.angleDeg());
        double radius_deg = _gps_rot_attack.radiusDeg();
        lat_offset += radius_deg * cos(angle_rad);
        lon_offset += radius_deg * sin(angle_rad);
    }

    double latitude = msg.latitude_deg() + lat_offset;
    double longitude = msg.longitude_deg() + lon_offset;
    double altitude = msg.altitude() + _gps_offset_attack.offsetAltM() + _gps_rot_attack.offsetAltM();

    float vel_north = msg.velocity_north();
    float vel_east = msg.velocity_east();
    float vel_down = -msg.velocity_up();

    // Publica a posicao global simulada resultante em topico auxiliar.
    vehicle_global_position_s gps_truth{};
    gps_truth.timestamp = timestamp;
    gps_truth.timestamp_sample = timestamp;
    gps_truth.lat = latitude;
    gps_truth.lon = longitude;
    gps_truth.alt = altitude;
    _gpos_ground_truth_pub.publish(gps_truth);

    // Ruido nominal do receptor GPS simulado.
    addGpsNoise(latitude, longitude, altitude, vel_north, vel_east, vel_down);

    // Ataque 1: jamming por ruido continuo. A perturbacao afeta posicao, altitude e velocidade.
    _jamming_attack.applyNoiseJamming(latitude, longitude, altitude, vel_north, vel_east, vel_down);

    // ======== MITIGACAO GPS - INICIO ========
    GpsMitigationContext gps_ctx = buildGpsMitigationContext();
    bool gps_anomalous_now = _gps_mitigation.detectAndCorrect(_attack_mitigation_enabled, _pos_ref, gps_ctx,
                          latitude, longitude, altitude, vel_north, vel_east, vel_down);

    // Propaga de volta a calibracao do offset barometrico (escrita por
    // detectAndCorrect no ramo nominal, lida pela mitigacao de barometro).
    _jmit_baro_alt_offset = gps_ctx.jmit_baro_alt_offset;
    _jmit_baro_alt_offset_valid = gps_ctx.jmit_baro_alt_offset_valid;
    // ======== MITIGACAO GPS - FIM ========

    // ======== MITIGACAO DE POUSO GPS - INICIO ========
    _gps_mitigation.checkAutoLand(_attack_mitigation_enabled, timestamp, _pos_ref, _alt_ref, gps_ctx, _gps_real_alt);

    _gps_mitigation.applyLandingOverride(timestamp, _pos_ref, _alt_ref, gps_ctx,
                          latitude, longitude, altitude,
                          vel_north, vel_east, vel_down,
                          gps_anomalous_now);

    _gps_mitigation.checkAutoDisarm(timestamp, gps_ctx);
    // ======== MITIGACAO DE POUSO GPS - FIM ========

    // Ataque 3: maquina de estados do jamming pulsado.
    _jamming_attack.tickPulsedJamming(hrt_absolute_time(), latitude, longitude, altitude,
                       vel_north, vel_east, vel_down);

    // Reproduz o "return" original do estado JAM (bloqueio total do sinal
    // enquanto o pulso esta ativo), exceto durante pouso automatico ja em
    // andamento - mesma condicao que existia dentro do switch original.
    if (_jamming_attack.isPulsedFullyJammed() && !_gps_mitigation.isLandingActive()) {
        return;
    }

    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
    id.devid_s.bus = 1;
    id.devid_s.address = 1;

    sensor_gps_s sensor_gps{};

    // Qualidade nominal do GPS quando ha satelites suficientes.
    if (_sim_gps_used.get() >= 4) {

        sensor_gps.fix_type = 3;
        sensor_gps.eph      = 0.9f;
        sensor_gps.epv      = 1.78f;
        sensor_gps.hdop     = 0.7f;
        sensor_gps.vdop     = 1.1f;
    } else {

        sensor_gps.fix_type = 0;
        sensor_gps.eph      = 100.f;
        sensor_gps.epv      = 100.f;
        sensor_gps.hdop     = 100.f;
        sensor_gps.vdop     = 100.f;
    }

    // Degrada eph/epv/hdop/vdop/satellites_used de acordo com o ataque de
    // jamming ativo (pulsado ou ruido continuo).
    _jamming_attack.degradeGpsQuality(sensor_gps, _sim_gps_used.get());


    sensor_gps.timestamp = timestamp;
    sensor_gps.timestamp_sample = timestamp;
    sensor_gps.device_id = id.devid;
    sensor_gps.latitude_deg = latitude;
    sensor_gps.longitude_deg = longitude;
    sensor_gps.altitude_msl_m = altitude;
    sensor_gps.altitude_ellipsoid_m = altitude;
    sensor_gps.vel_m_s = sqrtf(vel_north * vel_north + vel_east * vel_east);
    sensor_gps.vel_n_m_s = vel_north;
    sensor_gps.vel_e_m_s = vel_east;
    sensor_gps.vel_d_m_s = vel_down;
    // Durante o pouso ativo com velocidade zero, atan2(0,0)=0 diverge do heading real.
    // Usar o heading capturado mantem a estimativa do EKF consistente e previne
    // a 'Attitude failure (roll)' que causa o pouso inclinado.
    if (_gps_mitigation.isLandingActive() && _gps_mitigation.isLandingHeadingValid() &&
            fabsf(vel_north) < 0.05f && fabsf(vel_east) < 0.05f) {
        sensor_gps.cog_rad = _gps_mitigation.landingHeadingRad();
    } else {
        sensor_gps.cog_rad = atan2(vel_east, vel_north);
    }

    // ======== MITIGACAO GPS - INICIO ========
    // Em modo ancora, a pseudo-medicao GPS e marcada como degradada, mas continua publicada.
    const bool is_anchor_now = _gps_mitigation.isAnchorNow(_attack_mitigation_enabled);
    const bool landing_now = _gps_mitigation.isLandingNow();

    // A pseudo-GPS da mitigacao e publicada com qualidade operacional controlada.
    // Valores extremos de EPH/EPV/HDOP/VDOP podem acionar
    // health/failsafe antes do comando LAND controlado.
    if (is_anchor_now) {
        if (landing_now) {
            sensor_gps.eph  = 1.0f;
            sensor_gps.epv  = 1.5f;
            sensor_gps.hdop = 0.8f;
            sensor_gps.vdop = 1.0f;
        } else {
            sensor_gps.eph  = BLACKOUT_EPH_DEGRADE;
            sensor_gps.epv  = BLACKOUT_EPV_DEGRADE;
            sensor_gps.hdop = BLACKOUT_HDOP_DEGRADE;
            sensor_gps.vdop = BLACKOUT_VDOP_DEGRADE;
        }
    }
    // ======== MITIGACAO GPS - FIM ========

    // ======== INDICADORES DE GPS JAMMING E MITIGACAO - INICIO ========
    // Apenas amostras GPS reais anomalas recebem indicador de jamming.
    // A pseudo-medicao mitigada e tratada como saida operacional da contramedida.

    const bool is_jamming_now =
        gps_anomalous_now &&
        !is_anchor_now &&
        !landing_now;
    sensor_gps.jamming_indicator = is_jamming_now ? 255 : 0;
    sensor_gps.jamming_state     = is_jamming_now ? 3   : 0;
    // ======== INDICADORES DE GPS JAMMING E MITIGACAO - FIM ========

    sensor_gps.vel_ned_valid = true;

    // ======== MITIGACAO BLACKOUT GPS - INICIO ========
    // Este timestamp marca a chegada fisica de uma callback GPS ao bridge.
    // Ele deve ser atualizado mesmo em ancora/KF para evitar falso blackout em ABS, rotacao e noise.
    _gps_real_last_us = timestamp;
    _gps_real_valid = true;

    // Os valores limpos sao atualizados apenas fora da ancora e fora do
    // ruido do jamming pulsado. O ruido pulsado (tipo 3) e injetado depois
    // que a deteccao NIS do GPS ja rodou nesta mesma chamada, entao essa
    // deteccao nunca chega a ver esse ruido especifico — sem essa segunda
    // checagem, o valor contaminado passaria como "limpo" para quem usa
    // este cache como referencia independente (mitigacao de barometro,
    // entre outras).
    if (!is_anchor_now && !_jamming_attack.isPulsedIntermittentActive()) {
        float gps_n_m = 0.0f;
        float gps_e_m = 0.0f;
        _pos_ref.project(latitude, longitude, gps_n_m, gps_e_m);

        _gps_real_n_m = static_cast<double>(gps_n_m);
        _gps_real_e_m = static_cast<double>(gps_e_m);
        _gps_real_alt = altitude;
        _gps_real_alt_timestamp = timestamp;
        _gps_real_vel_n = vel_north;
        _gps_real_vel_e = vel_east;
        _gps_real_vel_d = vel_down;

        // ======== MITIGACAO IMU - INICIO ========
        // Aceleracao horizontal por diferenca finita da velocidade GPS, usada
        // pela mitigacao de IMU como referencia cruzada para o acelerometro.
        _imu_mitigation.updateGpsAccelReference(timestamp, vel_north, vel_east);
        // ======== MITIGACAO IMU - FIM ========
    }
    // ======== MITIGACAO BLACKOUT GPS - FIM ========

    _sensor_gps_pub.publish(sensor_gps);
}

// Publica sensor de distancia e aplica offset de ataque de LiDAR. Logica
// compartilhada pelas duas inscricoes possiveis (down e, em modelos
// combinados como o x500_uavjamsim, front) — is_front_slot diz qual delas
// chamou, para usar device address e instancia uORB de publicacao proprios,
// sem depender da orientacao detectada na mensagem (isso preserva o
// comportamento ja testado do x500_lidar_front standalone, que so tem a
// inscricao "down" mas fisicamente aponta para frente).
void GZBridge::laserScantoLidarSensorCallbackImpl(const gz::msgs::LaserScan &msg, bool is_front_slot)
{
    const uint64_t timestamp = hrt_absolute_time();

    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;
    id.devid_s.bus = 1;
    id.devid_s.address = is_front_slot ? 2 : 1;

    distance_sensor_s report{};
    report.timestamp = timestamp;
    report.device_id = id.devid;
    report.min_distance = LIDAR_RANGE_MIN_M;
    report.max_distance = LIDAR_RANGE_MAX_M;

    // Leitura bruta do primeiro feixe; o offset simula erro no sensor de distancia.
    // Sem retorno (nada no alcance), o valor bruto vem como infinito ou como
    // o proprio alcance maximo da mensagem (msg.range_max()): um offset
    // negativo aplicado direto sobre isso nunca produziria um numero finito
    // menor. Para o ataque conseguir forjar uma deteccao onde nao havia
    // nenhuma (cenario realista de spoofing), a leitura "sem retorno" e
    // tratada como o alcance maximo antes do offset ser somado.
    float raw_dist = static_cast<float>(msg.ranges()[0]);

    if (!PX4_ISFINITE(raw_dist) || (raw_dist >= static_cast<float>(msg.range_max()))) {
        raw_dist = LIDAR_RANGE_MAX_M;
    }

    float processed_dist = raw_dist;

    // Ataque LiDAR: soma offset a medicao de distancia.
    processed_dist = _lidar_attack.applyOffset(processed_dist);

    // Limita a medicao ao intervalo fisico aceito pelo PX4.
    report.current_distance = math::constrain(processed_dist, report.min_distance, report.max_distance);
    report.signal_quality = -1;

    report.variance = 0.0f;
    report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

    // A orientacao do sensor define se ele aponta para frente, para baixo ou em rotacao customizada.
    gz::msgs::Quaternion pose_orientation = msg.world_pose().orientation();
    gz::math::Quaterniond q_sensor = gz::math::Quaterniond(
            pose_orientation.w(), pose_orientation.x(), pose_orientation.y(), pose_orientation.z());

    const gz::math::Quaterniond q_front(0.7071068, 0.7071068, 0, 0);
    const gz::math::Quaterniond q_down(0, 1, 0, 0);

    if (q_sensor.Equal(q_front, 0.03)) {
        report.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
    } else if (q_sensor.Equal(q_down, 0.03)) {
        report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
    } else {
        report.orientation = distance_sensor_s::ROTATION_CUSTOM;
        report.q[0] = q_sensor.W();
        report.q[1] = q_sensor.X();
        report.q[2] = q_sensor.Y();
        report.q[3] = q_sensor.Z();
    }

    const bool touchdown_range_sensor =
        (report.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) ||
        (report.orientation == distance_sensor_s::ROTATION_CUSTOM);

    // ======== MITIGACAO LIDAR - INICIO ========
    if (touchdown_range_sensor) {
        _lidar_mitigation.detectAndCorrectDown(_attack_mitigation_enabled, timestamp,
                              _jmit_baro_valid, _jmit_baro_timestamp, _jmit_baro_alt_m,
                              report.min_distance, report.max_distance, report.current_distance);
    }
    // ======== MITIGACAO LIDAR - FIM ========

    // ======== MITIGACAO LIDAR FRONTAL - INICIO ========
    if (report.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) {
        _lidar_mitigation.detectAndCorrectFront(_attack_mitigation_enabled, timestamp,
                               report.min_distance, report.max_distance, report.current_distance);
    }
    // ======== MITIGACAO LIDAR FRONTAL - FIM ========

    if (touchdown_range_sensor && PX4_ISFINITE(report.current_distance)) {
        // This callback is subscribed to the vertical rangefinder topic.
        // Accept CUSTOM orientation as a valid Gazebo-mounted rangefinder.
        _ground_distance_m = report.current_distance;
        _ground_distance_valid = true;
        _ground_distance_timestamp = report.timestamp;
    }

    if (is_front_slot) {
        _distance_sensor_front_pub.publish(report);

    } else {
        _distance_sensor_pub.publish(report);
    }
}

// Callback da inscricao "down" (subscribeDistanceSensor). Em modelos com um
// unico sensor de distancia (down ou front), e essa inscricao quem recebe
// os dados, independente da orientacao fisica real do sensor.
void GZBridge::laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg)
{
    laserScantoLidarSensorCallbackImpl(msg, false);
}

// Callback da inscricao "front" adicional (subscribeDistanceSensorFront),
// usada apenas em modelos combinados (ex.: x500_uavjamsim) que tem down e
// front simultaneamente em links/sensores com nomes distintos.
void GZBridge::laserScantoLidarSensorFrontCallback(const gz::msgs::LaserScan &msg)
{
    laserScantoLidarSensorCallbackImpl(msg, true);
}

// Converte o LaserScan 2D em setores de distancia para prevencao de colisao.
void GZBridge::laserScanCallback(const gz::msgs::LaserScan &msg)
{
    // O PX4 espera distancias em setores angulares de 5 graus para prevencao de colisao.
    static constexpr int SECTOR_SIZE_DEG = 5;

    double angle_min_deg = msg.angle_min() * 180 / M_PI;
    double angle_step_deg = msg.angle_step() * 180 / M_PI;

    int samples_per_sector = std::round(SECTOR_SIZE_DEG / angle_step_deg);
    int number_of_sectors = msg.ranges_size() / samples_per_sector;

    std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

    // Reduz a resolucao do scan calculando a media das amostras de cada setor.
    for (int i = 0; i < number_of_sectors; i++) {

        double sum = 0;

        int samples_used_in_sector = 0;

        for (int j = 0; j < samples_per_sector; j++) {

            double distance = msg.ranges()[i * samples_per_sector + j];

            if (isinf(distance)) {
                continue;
            }

            // Reaproveita o mesmo comando/estado do ataque do LiDAR 1D
            // (--lidar 1 <offset>): soma o offset a cada feixe valido antes
            // da reducao em setores.
            distance = _lidar_attack.applyOffset(distance);

            sum += distance;
            samples_used_in_sector++;
        }

        if (samples_used_in_sector == 0) {
            ds_array[i] = msg.range_max();

        } else {
            ds_array[i] = sum / samples_used_in_sector;
        }
    }

    // Mensagem uORB usada pelo modulo de obstacle avoidance/collision prevention.
    obstacle_distance_s report {};

    for (auto &i : report.distances) {
        i = UINT16_MAX;
    }

    report.timestamp = hrt_absolute_time();
    report.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
    report.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
    report.min_distance = static_cast<uint16_t>(msg.range_min() * 100.);
    report.max_distance = static_cast<uint16_t>(msg.range_max() * 100.);
    report.angle_offset = static_cast<float>(angle_min_deg);
    report.increment = static_cast<float>(SECTOR_SIZE_DEG);

    int index = 0;

    // Inverte a ordem para converter o scan de FLU para FRD.
    for (std::vector<double>::reverse_iterator i = ds_array.rbegin(); i != ds_array.rend(); ++i) {

        // A comparacao de limites e feita em double, antes de qualquer
        // conversao para uint16_t. O ataque pode empurrar a leitura para um
        // valor negativo (offset negativo maior que a distancia real); um
        // double negativo convertido direto para uint16_t "estoura" e vira
        // um numero grande, caindo por acidente na faixa de "nada
        // detectado" — o oposto do que deveria significar fisicamente (algo
        // muito perto e o cenario mais critico para prevencao de colisao,
        // nao deve nunca ser confundido com ausencia de obstaculo).
        const double distance_cm_d = (*i) * 100.;

        if (!PX4_ISFINITE(distance_cm_d) || (distance_cm_d >= static_cast<double>(report.max_distance))) {
            report.distances[index] = report.max_distance + 1;

        } else if (distance_cm_d < static_cast<double>(report.min_distance)) {
            report.distances[index] = 0;

        } else {
            report.distances[index] = static_cast<uint16_t>(distance_cm_d);
        }

        index++;
    }

    // ======== MITIGACAO LIDAR 2D - INICIO ========
    _lidar_mitigation.detectAndCorrectTwoD(_attack_mitigation_enabled, report.timestamp,
                          report.angle_offset, report.increment,
                          report.max_distance, report.min_distance,
                          report.distances, static_cast<int>(sizeof(report.distances) / sizeof(report.distances[0])));
    // ======== MITIGACAO LIDAR 2D - FIM ========

    _obstacle_distance_pub.publish(report);
}

// Converte orientacao entre os referenciais FLU/ENU do Gazebo e FRD/NED do PX4.
void GZBridge::rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU)
{

    static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

    static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

    // Composicao final: corpo FLU/Gazebo para corpo FRD/PX4 no mundo NED.
    q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}

// Cria a instancia do modulo a partir dos argumentos de mundo e modelo.
int GZBridge::task_spawn(int argc, char *argv[])
{
    std::string world_name;
    std::string model_name;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "w:n:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'w':
            world_name = myoptarg;
            break;

        case 'n':
            model_name = myoptarg;
            break;

        default:
            print_usage();
            return PX4_ERROR;
        }
    }

    PX4_INFO("world: %s, model: %s", world_name.c_str(), model_name.c_str());

    GZBridge *instance = new GZBridge(world_name, model_name);

    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init() != PX4_OK) {
        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return PX4_ERROR;
    }

    return PX4_OK;
}

// Imprime o estado das interfaces de saida do modulo.
int GZBridge::print_status()
{
    PX4_INFO_RAW("ESC outputs:\n");
    _mixing_interface_esc.mixingOutput().printStatus();

    PX4_INFO_RAW("Servo outputs:\n");
    _mixing_interface_servo.mixingOutput().printStatus();

    PX4_INFO_RAW("Wheel outputs:\n");
    _mixing_interface_wheel.mixingOutput().printStatus();

    return 0;
}

// Trata comandos desconhecidos do modulo.
int GZBridge::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

// Imprime a ajuda de uso do modulo gz_bridge.
int GZBridge::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("gz_bridge", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
    PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Model name", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// Ponto de entrada C usado pelo PX4 para iniciar o modulo.
extern "C" __EXPORT int gz_bridge_main(int argc, char *argv[])
{
    return GZBridge::main(argc, argv);
}