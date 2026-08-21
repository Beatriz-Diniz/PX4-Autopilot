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
        subscribeDepthCamera(false);
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

// Inscreve a ponte na camera de profundidade (OakD-Lite), quando o modelo
// tiver uma (so o x500_uavjamsim). Usada como referencia cruzada
// independente pela mitigacao de LiDAR frontal e do LiDAR 2D.
//
// Usa o nome de topico FIXO ("/depth_camera", definido pela tag <topic> no
// sensor StereoOV7251 do model.sdf), nao o caminho hierarquico automatico
// (/world/.../sensor/StereoOV7251/depth_image). O sensor "depth_camera" do
// Gazebo so publica de fato no topico fixo quando a tag <topic> esta
// presente no SDF — o caminho hierarquico aparece listado (gz topic -l)
// mas nunca tem publicador ativo para esse tipo de sensor. Nao e limitacao
// de driver/GPU: a mesma maquina publica normalmente assim que o topico
// fixo e usado. Se o nome do topico no SDF mudar, essa string precisa
// mudar junto.
//
// Limitacao conhecida: por ser um nome fixo (nao escopado por mundo ou
// instancia de modelo), isso so funciona corretamente com um unico
// veiculo rodando por vez — varias instancias do x500_uavjamsim
// simultaneas colidiriam nesse mesmo topico.
bool GZBridge::subscribeDepthCamera(bool required)
{
    std::string depth_topic = "/depth_camera";

    if (!_node.Subscribe(depth_topic, &GZBridge::depthCameraCallback, this)) {
        PX4_WARN("failed to subscribe to %s", depth_topic.c_str());
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

    // if (!_node.Subscribe(gps_attack_topic, &GZBridge::gpsAttackCallback, this)) {
        // PX4_ERR("failed to subscribe to attack topic: %s", gps_attack_topic.c_str());
        // return required ? false : true;
    // }
    if (!_gps_offset_attack.init(gps_attack_topic)) {
        return required ? false : true;
    }

    // if (!_node.Subscribe(gps_rot_attack_topic, &GZBridge::gpsRotAttackCallback, this)) {
        // PX4_ERR("failed to subscribe to attack topic: %s", gps_rot_attack_topic.c_str());
        // return required ? false : true;
    // }
    if (!_gps_rot_attack.init(gps_rot_attack_topic)) {
        return required ? false : true;
    }

    if (!_node.Subscribe(imu_attack_topic, &GZBridge::imuAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", imu_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_node.Subscribe(motor_attack_topic, &GZBridge::motorAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", motor_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_node.Subscribe(mag_attack_topic, &GZBridge::magAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", mag_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_node.Subscribe(lidar_attack_topic, &GZBridge::lidarAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", lidar_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_node.Subscribe(stream_attack_topic, &GZBridge::streamAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", stream_attack_topic.c_str());
        return required ? false : true;
    }

    if (!_node.Subscribe(baro_attack_topic, &GZBridge::baroAttackCallback, this)) {
        PX4_ERR("failed to subscribe to attack topic: %s", baro_attack_topic.c_str());
        return required ? false : true;
    }

    // if (!_node.Subscribe(jamming_attack_topic, &GZBridge::jammingAttackCallback, this)) {
        // PX4_ERR("failed to subscribe to attack topic: %s", jamming_attack_topic.c_str());
        // return required ? false : true;
    // }
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

// ======== ATAQUE GPS OFFSET - INICIO ========
// Ataque GPS por offset: atualiza deslocamentos artificiais de latitude, longitude e altitude.
// void GZBridge::gpsAttackCallback(const gz::msgs::Vector3d &msg)
// {

    // msg.x/y/z representam offsets aplicados diretamente a latitude, longitude e altitude.
    // _gps_attack_offset.Set(msg.x(), msg.y(), msg.z());
// }
// ======== ATAQUE GPS OFFSET - FIM ========

// ======== ATAQUE GPS ROTACAO - INICIO ========
// Ataque GPS por rotacao: configura magnitude, angulo e offset vertical da perturbacao circular.
// void GZBridge::gpsRotAttackCallback(const gz::msgs::Vector3d &msg)
// {

    // msg.x define o raio em graus, msg.y o angulo e msg.z o deslocamento vertical.
    // _gps_attack_rot.Set(msg.x(), msg.y(), msg.z());
// }
// ======== ATAQUE GPS ROTACAO - FIM ========

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

// ======== ATAQUE MAGNETOMETRO - INICIO ========
// Ataque no magnetometro: seleciona o modo de alteracao aplicado durante a publicacao do sensor.
void GZBridge::magAttackCallback(const gz::msgs::Vector3d &msg)
{

    // A opcao e aplicada posteriormente em magnetometerCallback().
    _mag_attack_option = static_cast<int>(msg.x());
}
// ======== ATAQUE MAGNETOMETRO - FIM ========

// ======== ATAQUE LIDAR - INICIO ========
// Ataque no LiDAR: configura ativacao e offset de distancia em metros.
void GZBridge::lidarAttackCallback(const gz::msgs::Vector3d &msg)
{

    // msg.x habilita/desabilita o ataque; msg.y define o deslocamento de distancia.
    _lidar_attack_option = static_cast<int>(msg.x());
    _lidar_distance_offset = msg.y();

    PX4_INFO("Lidar Attack: Option=%d, Offset=%.2f", _lidar_attack_option, _lidar_distance_offset);
}
// ======== ATAQUE LIDAR - FIM ========

// ======== ATAQUE IMU - INICIO ========
// Ataque na IMU: habilita offsets, desabilitacao total ou limpeza das perturbacoes por eixo.
void GZBridge::imuAttackCallback(const gz::msgs::Vector3d &msg)
{
    int option = static_cast<int>(msg.x());
    int index = static_cast<int>(msg.y());
    double offset = msg.z();

    // Indices validos: 0..2 para acelerometro e 3..5 para giroscopio.
    if (index < 0 || index > 5) return;

    // Opcoes: 1 habilita offsets, 2 zera a IMU, 3 configura offset, 4 limpa, 5 desativa.
    switch (option) {
        case 1:
            _imu_attack_enabled = true;
            _imu_disabled = false;
            for (int i = 0; i < 6; i++) {
                _imu_active_offsets[i] = _imu_temp_offsets[i];
            }
            break;
        case 2:
            _imu_disabled = true;
            _imu_attack_enabled = false;
            break;
        case 3:
            _imu_temp_offsets[index] = offset;
            break;
        case 4:
            for (int i = 0; i < 6; i++) {
                _imu_temp_offsets[i] = 0.0;
                _imu_active_offsets[i] = 0.0;
            }
            break;
        case 5:
            _imu_attack_enabled = false;
            _imu_disabled = false;
            break;
    }
}
// ======== ATAQUE IMU - FIM ========

// ======== ATAQUE BAROMETRO - INICIO ========
// Ataque no barometro: configura offset de altitude que sera convertido em perturbacao de pressao.
void GZBridge::baroAttackCallback(const gz::msgs::Vector3d &msg)
{

    // msg.x habilita/desabilita o ataque; msg.y define offset de altitude em metros.
    _baro_attack_option = static_cast<int>(msg.x());
    _baro_alt_offset = msg.y();

    PX4_INFO("Baro Attack: Option=%d, Alt_Offset=%.2f m", _baro_attack_option, _baro_alt_offset);
}
// ======== ATAQUE BAROMETRO - FIM ========

// ======== ATAQUE GPS JAMMING - INICIO ========
// Ataque de jamming GPS: seleciona ruido continuo, blackout ou jamming pulsado.
// void GZBridge::jammingAttackCallback(const gz::msgs::Vector3d &msg)
// {
    // msg.x seleciona o tipo de jamming e msg.y define a intensidade.
    // _jamming_attack_type = static_cast<int>(msg.x());
    // _jamming_intensity   = static_cast<float>(msg.y());

    // ======== ATAQUE GPS JAMMING DESATIVADO - INICIO ========
    // Tipo 0: desativa o jamming e reinicia a maquina de estados pulsada.
    // if (_jamming_attack_type == 0) {

        // _pjam_state                = PulsedJamState::IDLE;
        // _pjam_state_enter_us       = 0;
        // _jamming_pulsed_initialized = false;
        // _jamming_intermittent_active = false;
        // _pjam_noise_factor         = 0.0f;
        // PX4_INFO("[Jamming] DISABLED");
    // ======== ATAQUE GPS JAMMING DESATIVADO - FIM ========

    // ======== ATAQUE 1 GPS NOISE JAMMING - INICIO ========
    // Tipo 1: jamming por ruido continuo aplicado a posicao, altitude e velocidade GPS.
    // } else if (_jamming_attack_type == 1) {
        // PX4_WARN("[Jamming] NOISE: ENABLED, Intensity=%.2f",
                 // static_cast<double>(_jamming_intensity));
    // ======== ATAQUE 1 GPS NOISE JAMMING - FIM ========

    // ======== ATAQUE 2 GPS BLACKOUT - INICIO ========
    // Tipo 2: blackout, simulando perda total de atualizacao GPS.
    // } else if (_jamming_attack_type == 2) {
        // PX4_WARN("[Jamming] BLACKOUT: ENABLED (Signal Drop)");
    // ======== ATAQUE 2 GPS BLACKOUT - FIM ========

    // ======== ATAQUE 3 GPS PULSED JAMMING - INICIO ========
    // Tipo 3: jamming pulsado com subida, bloqueio, decaimento e recuperacao.
    // } else if (_jamming_attack_type == 3) {

        // A intensidade controla a duracao do bloqueio total dentro do ciclo pulsado.
        // float clamped = math::constrain(_jamming_intensity, 0.1f, 1.0f);

        // _pjam_ramp_us  = 500000ULL;
        // _pjam_on_us    = static_cast<uint64_t>(clamped * 10.0f * 1e6f);
        // _pjam_decay_us = 800000ULL;
        // _pjam_off_us   = 2000000ULL;

        // if (!_jamming_pulsed_initialized) {
            // _pjam_state              = PulsedJamState::RAMP_UP;
            // _pjam_state_enter_us     = hrt_absolute_time();
            // _jamming_pulsed_initialized = true;
            // _jamming_intermittent_active = false;
            // _pjam_noise_factor       = 0.0f;
        // }

        // PX4_WARN("[Jamming] PULSED ENABLED | Intensity=%.2f"
                 // " | ramp=%llums on=%llums decay=%llums off=%llums",
                 // static_cast<double>(clamped),
                 // (unsigned long long)(_pjam_ramp_us  / 1000),
                 // (unsigned long long)(_pjam_on_us    / 1000),
                 // (unsigned long long)(_pjam_decay_us / 1000),
                 // (unsigned long long)(_pjam_off_us   / 1000));
    // }
    // ======== ATAQUE 3 GPS PULSED JAMMING - FIM ========
// }
// ======== ATAQUE GPS JAMMING - FIM ========

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

    _kf_initialized   = false;
    _kf_attack_dur_us = 0;
    _kf_attack_first  = true;
    _jmit_pub_n_m     = 0.0;
    _jmit_pub_e_m     = 0.0;
    _jmit_pub_alt     = 0.0;
    _jmit_pub_vel_n   = 0.0f;
    _jmit_pub_vel_e   = 0.0f;
    _jmit_pub_vel_d   = 0.0f;
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
    _sim_agl_ground_count = 0;

    _blackout_anchor_initialized = false;
    _blackout_start_us = 0;
    _blackout_last_pub_us = 0;
    _blackout_last_dr_us = 0;
    _blackout_detected = false;
    _gps_auto_land_sent = false;
    _gps_auto_disarm_sent = false;
    _gps_landing_active = false;
    _gps_land_source_blackout = false;
    _gps_land_source_severe_anomaly = false;
    _gps_auto_land_arrival_us = 0;
    _gps_auto_land_last_arrived_us = 0;
    _gps_land_hold_start_us = 0;
    _gps_land_hold_n_m = 0.0;
    _gps_land_hold_e_m = 0.0;
    _gps_land_start_alt_msl = 0.0;
    _gps_land_target_alt_msl = 0.0;
    _gps_land_last_alt_msl = 0.0;
    _gps_land_last_alt_valid = false;
    _gps_land_last_update_us = 0;
    _gps_anomaly_severe = false;
    _gps_anomaly_severe_us = 0;
    _gps_land_heading_rad = 0.0f;
    _gps_land_heading_valid = false;
    _lpos_heading_rad = 0.0f;

    // Reinicia o estado da mitigacao de IMU (deteccao, linha de base, pouso).
    _imu_baseline_initialized = false;
    _imu_accel_norm_ewma = 9.80665;
    _imu_accel_norm_var_ewma = 1.0;
    _imu_gyro_norm_ewma = 0.0;
    _imu_gyro_norm_var_ewma = 0.02;
    _imu_anomaly_active = false;
    _imu_anomaly_start_us = 0;
    _imu_recovery_start_us = 0;
    _imu_ever_armed = false;
    _imu_prev_armed = false;
    _imu_land_settle_until_us = 0;

    _imu_ref_gps_prev_valid = false;
    _imu_ref_gps_prev_us = 0;
    _imu_ref_gps_prev_vel_n = 0.0f;
    _imu_ref_gps_prev_vel_e = 0.0f;
    _imu_ref_gps_accel_valid = false;
    _imu_ref_gps_accel_us = 0;
    _imu_ref_gps_accel_n = 0.0f;
    _imu_ref_gps_accel_e = 0.0f;

    _vio_imu_gyro_valid = false;
    _vio_imu_gyro_us = 0;
    _vio_imu_accel_valid = false;
    _vio_imu_accel_us = 0;

    _imu_landing_active = false;
    _imu_auto_land_sent = false;
    _imu_auto_disarm_sent = false;
    _imu_auto_land_arrival_us = 0;
    _imu_auto_land_last_arrived_us = 0;
    _imu_land_hold_n_m = 0.0;
    _imu_land_hold_e_m = 0.0;
    _imu_sim_agl_ground_count = 0;

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
    // base, pouso).
    _mag_baseline_initialized = false;
    _mag_baseline_warmup_count = 0;
    _mag_heading_offset_rad = 0.0;
    _mag_anomaly_active = false;
    _mag_anomaly_start_us = 0;
    _mag_recovery_start_us = 0;
    _mag_mitig_diag_count = 0;
    _mag_landing_active = false;
    _mag_auto_land_sent = false;
    _mag_auto_disarm_sent = false;
    _mag_auto_land_arrival_us = 0;
    _mag_auto_land_last_arrived_us = 0;
    _mag_land_hold_n_m = 0.0;
    _mag_land_hold_e_m = 0.0;
    _mag_sim_agl_ground_count = 0;

    // Reinicia o estado da mitigacao de LiDAR (deteccao, ancora, pouso).
    _lidar_baseline_initialized = false;
    _lidar_baseline_warmup_count = 0;
    _lidar_anchor_offset_m = 0.0;
    _lidar_anomaly_active = false;
    _lidar_anomaly_start_us = 0;
    _lidar_recovery_start_us = 0;
    _lidar_mitig_diag_count = 0;
    _lidar_landing_active = false;
    _lidar_auto_land_sent = false;
    _lidar_auto_disarm_sent = false;
    _lidar_auto_land_arrival_us = 0;
    _lidar_auto_land_last_arrived_us = 0;
    _lidar_land_hold_n_m = 0.0;
    _lidar_land_hold_e_m = 0.0;
    _lidar_sim_agl_ground_count = 0;

    // Reinicia o estado da mitigacao de LiDAR frontal (deteccao, pouso).
    // O cache da camera de profundidade (_depth_cam_*) nao e resetado aqui:
    // ele nao depende do estado da mitigacao, so reflete a leitura mais
    // recente do sensor, igual outros caches de referencia do projeto.
    _lidar_front_anomaly_active = false;
    _lidar_front_anomaly_start_us = 0;
    _lidar_front_recovery_start_us = 0;
    _lidar_front_mitig_diag_count = 0;
    _lidar_front_landing_active = false;
    _lidar_front_auto_land_sent = false;
    _lidar_front_auto_disarm_sent = false;
    _lidar_front_auto_land_arrival_us = 0;
    _lidar_front_auto_land_last_arrived_us = 0;
    _lidar_front_land_hold_n_m = 0.0;
    _lidar_front_land_hold_e_m = 0.0;
    _lidar_front_sim_agl_ground_count = 0;

    // Reinicia o estado da mitigacao de LiDAR 2D (deteccao, pouso). O cache
    // por setor (_depth_cam_sector_*) nao e resetado pelo mesmo motivo do
    // cache central acima.
    _lidar2d_anomaly_active = false;
    _lidar2d_anomaly_start_us = 0;
    _lidar2d_recovery_start_us = 0;
    _lidar2d_mitig_diag_count = 0;
    _lidar2d_landing_active = false;
    _lidar2d_auto_land_sent = false;
    _lidar2d_auto_disarm_sent = false;
    _lidar2d_auto_land_arrival_us = 0;
    _lidar2d_auto_land_last_arrived_us = 0;
    _lidar2d_land_hold_n_m = 0.0;
    _lidar2d_land_hold_e_m = 0.0;
    _lidar2d_sim_agl_ground_count = 0;

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

    // ======== ATAQUE MAGNETOMETRO - INICIO ========
    // Ataque de magnetometro: opcao 1 troca os eixos X e Y.
    switch (_mag_attack_option) {
        case 1:
        report.x = raw_y;
        report.y = raw_x;
        report.z = raw_z;

        break;

    default:
        report.x = raw_x;
        report.y = raw_y;
        report.z = raw_z;
        break;
    }
    // ======== ATAQUE MAGNETOMETRO - FIM ========

    // ======== MITIGACAO MAGNETOMETRO - INICIO ========
    // Deteccao de anomalia no sinal publicado do sensor, sem consultar
    // _mag_attack_option:
    //  (a) magnitude do campo horizontal fora de uma faixa fisica plausivel;
    //  (b) heading implicito no magnetometro divergindo do heading da VIO
    //      (referencia independente) em relacao a um offset calibrado a
    //      partir das primeiras amostras saudaveis e adaptado lentamente depois.
    // So confia no calculo de heading quando o veiculo esta proximo do
    // nivelado (roll/pitch da VIO abaixo de MAG_MAX_TILT_RAD): a convencao de
    // eixos do magnetometro simulado nao e garantida como a mesma FRD usada
    // por accel/gyro, entao nao arriscamos compensar inclinacao via rotacao.
    // Nao exige que o veiculo ja tenha armado/voado: o heading e valido
    // (e ate mais facil de validar, ja que vio_near_level tende a ser
    // verdadeiro parado no chao) mesmo antes da decolagem, e o ataque deve
    // ser detectavel nesse estado.
    if (_attack_mitigation_enabled) {

        const double horiz_mag = sqrt(
            static_cast<double>(report.x) * static_cast<double>(report.x) +
            static_cast<double>(report.y) * static_cast<double>(report.y));

        const bool horiz_out_of_bounds =
            !PX4_ISFINITE(horiz_mag) ||
            (horiz_mag < MAG_FIELD_HORIZ_MIN_G) ||
            (horiz_mag > MAG_FIELD_HORIZ_MAX_G);

        const bool vio_att_recent =
            _vio_heading_valid &&
            (timestamp >= _vio_heading_us) &&
            ((timestamp - _vio_heading_us) <= MAG_VIO_MAX_AGE_US);

        const bool vio_near_level =
            vio_att_recent &&
            (fabsf(_vio_roll_rad) < MAG_MAX_TILT_RAD) &&
            (fabsf(_vio_pitch_rad) < MAG_MAX_TILT_RAD);

        const bool heading_ref_valid = vio_near_level && (horiz_mag > 1e-9);

        double mag_heading = 0.0;
        double offset = 0.0;
        double heading_residual = 0.0;
        bool heading_anomaly = false;

        if (heading_ref_valid) {
            mag_heading = atan2(static_cast<double>(report.y), static_cast<double>(report.x));

            // A relacao real entre o heading implicito no magnetometro e o
            // heading da VIO e uma reflexao (mag_heading ~= -vio_heading + c),
            // nao uma simples rotacao (mag_heading ~= vio_heading + c). Isso
            // vem do mapeamento de eixos do magnetometro simulado, que tem
            // determinante -1 (troca e inverte os dois eixos horizontais).
            offset = atan2(sin(mag_heading + static_cast<double>(_vio_heading_rad)),
                     cos(mag_heading + static_cast<double>(_vio_heading_rad)));

            if (_mag_baseline_initialized) {
                heading_residual = atan2(sin(offset - _mag_heading_offset_rad), cos(offset - _mag_heading_offset_rad));
                heading_anomaly = fabs(heading_residual) > MAG_HEADING_ERROR_THRESHOLD_RAD;
            }
        }

        const bool sample_anomalous = horiz_out_of_bounds || heading_anomaly;

        // Severo (confirmacao rapida) somente para violacao fisica dura. Um
        // desacordo de heading, por maior que seja, pode vir de interferencia
        // magnetica transitoria durante manobra (curva com tracao diferencial
        // entre motores), entao sempre passa pela janela de confirmacao longa
        // (MAG_ANOMALY_CONFIRM_US) em vez do caminho rapido.
        const bool sample_severe = horiz_out_of_bounds;

        if (!sample_anomalous) {

            // Atualiza a linha de base apenas com amostras saudaveis. As
            // primeiras amostras (warmup) calibram o offset com um alpha mais
            // agressivo, para nao travar numa unica amostra ruidosa logo no
            // inicio do voo real; depois disso, adapta lentamente (tolera
            // deriva legitima do sensor, mas um ataque sustentado nao chega a
            // ser absorvido, pois so entra aqui quando a amostra nao e anomala).
            if (heading_ref_valid) {
                if (_mag_baseline_warmup_count == 0) {
                    _mag_heading_offset_rad = offset;
                    _mag_baseline_warmup_count = 1;

                } else if (_mag_baseline_warmup_count < MAG_BASELINE_WARMUP_SAMPLES) {
                    const double delta = atan2(sin(offset - _mag_heading_offset_rad), cos(offset - _mag_heading_offset_rad));
                    _mag_heading_offset_rad = atan2(
                        sin(_mag_heading_offset_rad + MAG_BASELINE_WARMUP_ALPHA * delta),
                        cos(_mag_heading_offset_rad + MAG_BASELINE_WARMUP_ALPHA * delta));
                    _mag_baseline_warmup_count++;

                    if (_mag_baseline_warmup_count >= MAG_BASELINE_WARMUP_SAMPLES) {
                        _mag_baseline_initialized = true;
                    }

                } else {
                    const double delta = atan2(sin(offset - _mag_heading_offset_rad), cos(offset - _mag_heading_offset_rad));
                    _mag_heading_offset_rad = atan2(
                        sin(_mag_heading_offset_rad + MAG_BASELINE_ALPHA * delta),
                        cos(_mag_heading_offset_rad + MAG_BASELINE_ALPHA * delta));
                }
            }

            if (_mag_anomaly_active) {
                if (_mag_recovery_start_us == 0) {
                    _mag_recovery_start_us = timestamp;

                } else if ((timestamp - _mag_recovery_start_us) >= MAG_RECOVERY_CONFIRM_US) {
                    _mag_anomaly_active = false;
                    _mag_anomaly_start_us = 0;
                    _mag_mitig_diag_count = 0;

                    PX4_INFO("\n[Mag-Mitig] Sensor anomaly cleared\n");
                }
            }

        } else {
            _mag_recovery_start_us = 0;

            if (_mag_anomaly_start_us == 0) {
                _mag_anomaly_start_us = timestamp;
            }

            const uint64_t mag_confirm_required_us =
                sample_severe ? MAG_ANOMALY_CONFIRM_SEVERE_US : MAG_ANOMALY_CONFIRM_US;

            if (!_mag_anomaly_active &&
                    ((timestamp - _mag_anomaly_start_us) >= mag_confirm_required_us)) {
                _mag_anomaly_active = true;

                PX4_WARN("\n[Mag-Mitig] Sensor anomaly detected | heading_residual=%.1f deg | mag_heading=%.1f deg | vio_heading=%.1f deg | offset=%.1f deg | baseline=%.1f deg | bounds=%s\n",
                    math::degrees(heading_residual),
                    math::degrees(mag_heading),
                    static_cast<double>(math::degrees(_vio_heading_rad)),
                    math::degrees(offset),
                    math::degrees(_mag_heading_offset_rad),
                    horiz_out_of_bounds ? "out" : "ok");
            }

            // Reconstroi a direcao a partir do heading esperado (reflexao do
            // heading da VIO mais o offset calibrado, mesmo modelo da
            // deteccao), preservando a magnitude horizontal ainda confiavel
            // do proprio sinal atacado. So roda quando a ultima referencia de
            // heading valida foi calculada em voo nivelado (heading_ref_valid
            // so fica true nessas condicoes).
            if (_mag_anomaly_active && heading_ref_valid) {
                const double expected_heading = atan2(
                    sin(_mag_heading_offset_rad - static_cast<double>(_vio_heading_rad)),
                    cos(_mag_heading_offset_rad - static_cast<double>(_vio_heading_rad)));

                report.x = static_cast<float>(horiz_mag * cos(expected_heading));
                report.y = static_cast<float>(horiz_mag * sin(expected_heading));
            }

            if (_mag_anomaly_active && (_mag_mitig_diag_count < MAG_MITIG_DIAG_MAX_COUNT)) {
                static uint64_t mag_mitig_diag_last_us = 0;

                if ((timestamp - mag_mitig_diag_last_us) > 1000000ULL) {
                    mag_mitig_diag_last_us = timestamp;
                    _mag_mitig_diag_count++;

                    PX4_INFO("\n[Mag-Mitig] Status | heading_residual=%.1f deg | mag_heading=%.1f deg | vio_heading=%.1f deg | horiz_mag=%.2e G | vio_age=%.0f ms\n",
                        math::degrees(heading_residual),
                        math::degrees(mag_heading),
                        static_cast<double>(math::degrees(_vio_heading_rad)),
                        horiz_mag,
                        _vio_heading_valid ? static_cast<double>(timestamp - _vio_heading_us) / 1000.0 : -1.0);

                    if (_mag_mitig_diag_count == MAG_MITIG_DIAG_MAX_COUNT) {
                        PX4_WARN("\n[Mag-Mitig] Check: listener vehicle_magnetometer\n");
                    }
                }
            }
        }
    }
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

    // ======== ATAQUE BAROMETRO - INICIO ========
    // Ataque barometrico: converte offset de altitude em alteracao fisica de pressao.
    if (_baro_attack_option == 1) {

        float altitude_offset_m = static_cast<float>(_baro_alt_offset);

        float temperature_k = raw_temperature + 273.15f;

        float adjusted_temperature_k = temperature_k - (0.0065f * altitude_offset_m);

        float pressure_ratio = powf(adjusted_temperature_k / temperature_k, 5.2561f);

        report.pressure = raw_pressure * pressure_ratio;
        report.temperature = adjusted_temperature_k - 273.15f;
    } else {
        report.pressure = raw_pressure;
        report.temperature = raw_temperature;
    }
    // ======== ATAQUE BAROMETRO - FIM ========

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

    // ======== ATAQUE IMU - INICIO ========
    // Ataque IMU: pode zerar completamente o sensor ou somar offsets configurados por eixo.
    if (_imu_disabled) {
        accel.x = 0.0f; accel.y = 0.0f; accel.z = 0.0f;
        gyro.x = 0.0f; gyro.y = 0.0f; gyro.z = 0.0f;
    } else if (_imu_attack_enabled) {

        accel.x += static_cast<float>(_imu_active_offsets[0]);
        accel.y += static_cast<float>(_imu_active_offsets[1]);
        accel.z += static_cast<float>(_imu_active_offsets[2]);
        gyro.x += static_cast<float>(_imu_active_offsets[3]);
        gyro.y += static_cast<float>(_imu_active_offsets[4]);
        gyro.z += static_cast<float>(_imu_active_offsets[5]);
    }
    // ======== ATAQUE IMU - FIM ========

    // ======== MITIGACAO IMU - INICIO ========
    // Deteccao de anomalia no sinal publicado do sensor (accel/gyro ja com o
    // ataque aplicado, se houver). Combina tres verificacoes:
    //  (1) Limites fisicos de forca especifica.
    //  (2) Z-score contra uma linha de base (media do acelerometro fixa na
    //      gravidade local; media/variancia do giroscopio adaptativas).
    //  (3) Aceleracao horizontal medida pela IMU vs. aceleracao horizontal
    //      estimada por diferenca finita da velocidade GPS.
    
    // A deteccao so roda depois que o veiculo esteve armado e voando (nao
    // pousado) pela primeira vez desde que a mitigacao foi habilitada.
    actuator_armed_s imu_actuator_armed{};
    const bool imu_armed_available = _actuator_armed_sub.copy(&imu_actuator_armed);
    const bool imu_vehicle_armed = imu_armed_available && imu_actuator_armed.armed;

    vehicle_land_detected_s imu_land_detected{};
    const bool imu_land_detector_available = _vehicle_land_detected_sub.copy(&imu_land_detected);
    const bool imu_vehicle_not_landed = imu_land_detector_available && !imu_land_detected.landed;

    if (imu_vehicle_armed && imu_vehicle_not_landed) {
        _imu_ever_armed = true;
    }

    // Suspende a deteccao por uma janela curta na transicao de armado para
    // desarmado: o corte do empuxo pode gerar uma leitura transitoria de
    // forca especifica proxima de zero, fisicamente identica a uma IMU
    // zerada, sem relacao com ataque.
    if (!imu_vehicle_armed && _imu_prev_armed) {
        _imu_land_settle_until_us = timestamp + IMU_LAND_SETTLE_US;
    }

    _imu_prev_armed = imu_vehicle_armed;

    if (_attack_mitigation_enabled && _imu_ever_armed && (timestamp >= _imu_land_settle_until_us)) {

        const double accel_x_d = static_cast<double>(accel.x);
        const double accel_y_d = static_cast<double>(accel.y);
        const double accel_z_d = static_cast<double>(accel.z);
        const double gyro_x_d  = static_cast<double>(gyro.x);
        const double gyro_y_d  = static_cast<double>(gyro.y);
        const double gyro_z_d  = static_cast<double>(gyro.z);

        const double accel_norm = sqrt(
            accel_x_d * accel_x_d +
            accel_y_d * accel_y_d +
            accel_z_d * accel_z_d);

        const double gyro_norm = sqrt(
            gyro_x_d * gyro_x_d +
            gyro_y_d * gyro_y_d +
            gyro_z_d * gyro_z_d);

        // (1) Limites fisicos de forca especifica.
        const bool accel_out_of_bounds =
            !PX4_ISFINITE(accel_norm) ||
            (accel_norm < IMU_ACCEL_NORM_MIN_MS2) ||
            (accel_norm > IMU_ACCEL_NORM_MAX_MS2);

        // (2) Z-score contra a linha de base.
        const double accel_std = sqrt(math::max(_imu_accel_norm_var_ewma, 1e-6));
        const double accel_z = fabs(accel_norm - _imu_accel_norm_ewma) / accel_std;

        double gyro_z = 0.0;

        if (_imu_baseline_initialized) {
            const double gyro_std = sqrt(math::max(_imu_gyro_norm_var_ewma, 1e-6));
            gyro_z = fabs(gyro_norm - _imu_gyro_norm_ewma) / gyro_std;
        }

        const bool statistical_anomaly =
            (accel_z > IMU_ZSCORE_THRESHOLD) ||
            (_imu_baseline_initialized && (gyro_z > IMU_ZSCORE_THRESHOLD));

        // (3) Aceleracao horizontal da IMU vs. aceleracao horizontal do GPS.
        // So se aplica quando a referencia de GPS esta recente.
        const bool gps_ref_recent =
            _imu_ref_gps_accel_valid &&
            (timestamp >= _imu_ref_gps_accel_us) &&
            ((timestamp - _imu_ref_gps_accel_us) <= IMU_REF_GPS_MAX_AGE_US);

        double gps_crosscheck_residual = 0.0;
        bool gps_crosscheck_anomaly = false;

        if (gps_ref_recent) {
            const double accel_horiz = sqrt(accel_x_d * accel_x_d + accel_y_d * accel_y_d);

            const double gps_accel_n_d = static_cast<double>(_imu_ref_gps_accel_n);
            const double gps_accel_e_d = static_cast<double>(_imu_ref_gps_accel_e);
            const double gps_horiz = sqrt(gps_accel_n_d * gps_accel_n_d + gps_accel_e_d * gps_accel_e_d);

            gps_crosscheck_residual = fabs(accel_horiz - gps_horiz);
            gps_crosscheck_anomaly = gps_crosscheck_residual > IMU_CROSSCHECK_RESIDUAL_MAX_MS2;
        }

        const bool sample_anomalous =
            accel_out_of_bounds || statistical_anomaly || gps_crosscheck_anomaly;

        // Anomalia severa: violacao fisica dura (limites/GPS) ou z-score acima
        // de IMU_ZSCORE_SEVERE_THRESHOLD. Usa a janela de confirmacao reduzida.
        const bool sample_severe =
            accel_out_of_bounds ||
            gps_crosscheck_anomaly ||
            (accel_z > IMU_ZSCORE_SEVERE_THRESHOLD) ||
            (_imu_baseline_initialized && (gyro_z > IMU_ZSCORE_SEVERE_THRESHOLD));

        // Log periodico de diagnostico, apenas enquanto a anomalia esta ativa.
        if (_imu_anomaly_active) {
            static uint64_t imu_mitig_diag_last_us = 0;

            if ((timestamp - imu_mitig_diag_last_us) > 1000000ULL) {
                imu_mitig_diag_last_us = timestamp;

                PX4_INFO("\n[IMU-Mitig] Status | accel_norm=%.2f (z=%.1f) gyro_norm=%.3f (z=%.1f) | "
                    "gps_ref=%s residual=%.2f\n",
                    accel_norm, accel_z, gyro_norm, gyro_z,
                    gps_ref_recent ? "ok" : "stale",
                    gps_crosscheck_residual);
            }
        }

        if (!sample_anomalous) {

            // Atualiza a linha de base com a amostra saudavel. A media do
            // acelerometro permanece fixa; variancia do acelerometro e
            // media/variancia do giroscopio se adaptam.
            if (!_imu_baseline_initialized) {
                _imu_gyro_norm_ewma  = gyro_norm;
                _imu_accel_norm_var_ewma = 1.0;
                _imu_gyro_norm_var_ewma  = 0.1;
                _imu_baseline_initialized = true;

            } else {
                const double d_accel = accel_norm - _imu_accel_norm_ewma;
                const double d_gyro  = gyro_norm - _imu_gyro_norm_ewma;

                _imu_gyro_norm_ewma += IMU_BASELINE_ALPHA * d_gyro;

                _imu_accel_norm_var_ewma =
                    (1.0 - IMU_BASELINE_ALPHA) * _imu_accel_norm_var_ewma +
                    IMU_BASELINE_ALPHA * d_accel * d_accel;

                _imu_gyro_norm_var_ewma =
                    (1.0 - IMU_BASELINE_ALPHA) * _imu_gyro_norm_var_ewma +
                    IMU_BASELINE_ALPHA * d_gyro * d_gyro;

                _imu_accel_norm_var_ewma = math::constrain(_imu_accel_norm_var_ewma, 0.05, 4.0);
                _imu_gyro_norm_var_ewma  = math::constrain(_imu_gyro_norm_var_ewma, 0.02, 2.0);
            }

            if (_imu_anomaly_active) {
                // Confirma recuperacao apos um intervalo estavel sem anomalia.
                if (_imu_recovery_start_us == 0) {
                    _imu_recovery_start_us = timestamp;

                } else if ((timestamp - _imu_recovery_start_us) >= IMU_RECOVERY_CONFIRM_US) {
                    _imu_anomaly_active = false;
                    _imu_anomaly_start_us = 0;

                    PX4_INFO("\n[IMU-Mitig] Sensor anomaly cleared | accel_norm=%.2f gyro_norm=%.3f\n",
                        accel_norm, gyro_norm);
                }
            }

        } else {

            _imu_recovery_start_us = 0;

            if (_imu_anomaly_start_us == 0) {
                _imu_anomaly_start_us = timestamp;
            }

            const uint64_t confirm_required_us =
                sample_severe ? IMU_ANOMALY_CONFIRM_SEVERE_US : IMU_ANOMALY_CONFIRM_US;

            if (!_imu_anomaly_active &&
                    ((timestamp - _imu_anomaly_start_us) >= confirm_required_us)) {
                _imu_anomaly_active = true;

                PX4_WARN("\n[IMU-Mitig] Sensor anomaly detected | accel_norm=%.2f (z=%.1f) gyro_norm=%.3f (z=%.1f) | "
                    "bounds=%s gps_residual=%.2f (%s) | severe=%s confirm=%.0fms\n",
                    accel_norm, accel_z, gyro_norm, gyro_z,
                    accel_out_of_bounds ? "out" : "ok",
                    gps_crosscheck_residual,
                    gps_crosscheck_anomaly ? "anomalous" : (gps_ref_recent ? "ok" : "stale"),
                    sample_severe ? "yes" : "no",
                    static_cast<double>(confirm_required_us) / 1000.0);
            }

            // Substitui accel/gyro pelo substituto dinamico reconstruido a
            // partir da VIO (ver bloco de reconstrucao em odometryCallback).
            // Uma amostra severa (violacao fisica dura ou z-score muito acima
            // do normal) e substituida imediatamente, mesmo antes da anomalia
            // ser oficialmente confirmada: isso evita que qualquer amostra
            // obviamente atacada chegue crua ao EKF do PX4 durante a janela
            // de confirmacao. A confirmacao oficial (log/pouso) continua
            // exigindo a janela de debounce normalmente.
            if (_imu_anomaly_active || sample_severe) {
                const bool vio_accel_recent =
                    _vio_imu_accel_valid &&
                    (timestamp >= _vio_imu_accel_us) &&
                    ((timestamp - _vio_imu_accel_us) <= IMU_VIO_SUBSTITUTE_MAX_AGE_US);

                const bool vio_gyro_recent =
                    _vio_imu_gyro_valid &&
                    (timestamp >= _vio_imu_gyro_us) &&
                    ((timestamp - _vio_imu_gyro_us) <= IMU_VIO_SUBSTITUTE_MAX_AGE_US);

                if (vio_accel_recent) {
                    accel.x = _vio_imu_accel_body[0] + generate_wgn() * IMU_VIO_SUBSTITUTE_ACCEL_NOISE_STD;
                    accel.y = _vio_imu_accel_body[1] + generate_wgn() * IMU_VIO_SUBSTITUTE_ACCEL_NOISE_STD;
                    accel.z = _vio_imu_accel_body[2] + generate_wgn() * IMU_VIO_SUBSTITUTE_ACCEL_NOISE_STD;
                }

                if (vio_gyro_recent) {
                    gyro.x = _vio_imu_gyro_body[0] + generate_wgn() * IMU_VIO_SUBSTITUTE_GYRO_NOISE_STD;
                    gyro.y = _vio_imu_gyro_body[1] + generate_wgn() * IMU_VIO_SUBSTITUTE_GYRO_NOISE_STD;
                    gyro.z = _vio_imu_gyro_body[2] + generate_wgn() * IMU_VIO_SUBSTITUTE_GYRO_NOISE_STD;
                }

                static uint64_t vio_substitute_log_last_us = 0;

                if ((timestamp - vio_substitute_log_last_us) > 1000000ULL) {
                    vio_substitute_log_last_us = timestamp;

                    PX4_INFO("\n[IMU-Mitig] VIO substitute | accel=%s gyro=%s\n",
                        vio_accel_recent ? "active" : "stale",
                        vio_gyro_recent ? "active" : "stale");
                }
            }
        }
    }
    // ======== MITIGACAO IMU - FIM ========

    _sensor_accel_pub.publish(accel);
    _sensor_gyro_pub.publish(gyro);

    if (_gps_landing_active || _gps_auto_land_sent) {
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
    // A mitigacao detecta blackout por sintoma: o GPS real ficou sem publicacao recente,
    // enquanto o restante do sistema continua executando. A condicao e definida pelo timeout do GPS real.
    const bool gps_real_recent =
        _gps_real_valid &&
        ((timestamp - _gps_real_last_us) <= BLACKOUT_GPS_TIMEOUT_US);

    const bool blackout_detected =
        _attack_mitigation_enabled &&
        !_gps_auto_disarm_sent &&
        _pos_ref.isInitialized() &&
        _gps_real_valid &&
        !gps_real_recent;

    if (blackout_detected) {

        const bool lpos_recent =
            _lpos_xy_valid &&
            _lpos_z_valid &&
            (timestamp >= _lpos_timestamp) &&
            ((timestamp - _lpos_timestamp) <= 500000ULL) &&
            PX4_ISFINITE(_lpos_n_m) &&
            PX4_ISFINITE(_lpos_e_m) &&
            PX4_ISFINITE(_lpos_alt_msl);

        const bool vio_recent =
            _vio_valid &&
            (timestamp >= _vio_timestamp) &&
            ((timestamp - _vio_timestamp) <= BLACKOUT_VIO_TIMEOUT_US);

        if (!_blackout_anchor_initialized) {
            // Inicializa a ancora usando a melhor estimativa local recente.
            // Durante blackout, a pseudo-medicao precisa acompanhar a navegacao ate o destino.
            const char *anchor_source = "LAST_GPS";

            if (vio_recent) {
                _blackout_anchor_n_m = _vio_n_m;
                _blackout_anchor_e_m = _vio_e_m;
                _blackout_anchor_alt = _vio_alt_msl;
                _blackout_dr_vel_n   = _vio_vel_n;
                _blackout_dr_vel_e   = _vio_vel_e;
                _blackout_dr_vel_d   = 0.0f;
                anchor_source = "VIO";

            } else if (lpos_recent) {
                _blackout_anchor_n_m = _lpos_n_m;
                _blackout_anchor_e_m = _lpos_e_m;
                _blackout_anchor_alt = _lpos_alt_msl;
                _blackout_dr_vel_n   = _imu_dr_vel_n;
                _blackout_dr_vel_e   = _imu_dr_vel_e;
                _blackout_dr_vel_d   = 0.0f;
                anchor_source = "LPOS";

            } else if (_kf_initialized) {
                _blackout_anchor_n_m = _kf_n_m;
                _blackout_anchor_e_m = _kf_e_m;
                _blackout_anchor_alt = _kf_alt;
                _blackout_dr_vel_n   = _kf_vel_n;
                _blackout_dr_vel_e   = _kf_vel_e;
                _blackout_dr_vel_d   = 0.0f;
                anchor_source = "KF";

            } else {
                _blackout_anchor_n_m = _gps_real_n_m;
                _blackout_anchor_e_m = _gps_real_e_m;
                _blackout_anchor_alt = _gps_real_alt;
                _blackout_dr_vel_n   = _gps_real_vel_n;
                _blackout_dr_vel_e   = _gps_real_vel_e;
                _blackout_dr_vel_d   = 0.0f;
            }

            _blackout_dr_n_m      = _blackout_anchor_n_m;
            _blackout_dr_e_m      = _blackout_anchor_e_m;
            _blackout_dr_alt      = _blackout_anchor_alt;
            _blackout_start_us    = timestamp;
            _blackout_last_dr_us  = timestamp;
            _blackout_last_pub_us = 0;
            _blackout_anchor_initialized = true;
            _blackout_detected = true;

            PX4_WARN("\n[Blackout-Mitig] Blackout detected by GPS timeout: %.0f ms | anchor N=%.1f E=%.1f Alt=%.1f | source=%s\n",
                static_cast<double>(timestamp - _gps_real_last_us) / 1000.0,
                _blackout_dr_n_m, _blackout_dr_e_m, _blackout_dr_alt,
                anchor_source);
        }

        if (vio_recent) {
            // Fonte principal no blackout: odometria visual.
            // Isso evita que a pseudo-GPS fique parada enquanto o modelo se move fisicamente.
            _blackout_dr_n_m   = _vio_n_m;
            _blackout_dr_e_m   = _vio_e_m;
            _blackout_dr_alt   = _vio_alt_msl;
            _blackout_dr_vel_n = _vio_vel_n;
            _blackout_dr_vel_e = _vio_vel_e;
            _blackout_dr_vel_d = _vio_vel_d;
            _blackout_last_dr_us = timestamp;

        } else if (lpos_recent) {
            // Fallback: estimativa local do PX4, usada apenas se VIO nao estiver recente.
            _blackout_dr_n_m   = _lpos_n_m;
            _blackout_dr_e_m   = _lpos_e_m;
            _blackout_dr_alt   = _lpos_alt_msl;
            _blackout_dr_vel_n = _imu_dr_vel_n;
            _blackout_dr_vel_e = _imu_dr_vel_e;
            _blackout_dr_vel_d = _imu_dr_vel_d;
            _blackout_last_dr_us = timestamp;

        } else {
            const double dt_dr = math::constrain(
                static_cast<double>(timestamp - _blackout_last_dr_us) * 1e-6,
                0.0,
                0.1
            );

            _blackout_last_dr_us = timestamp;

            _blackout_dr_vel_n = math::constrain(_blackout_dr_vel_n, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S);
            _blackout_dr_vel_e = math::constrain(_blackout_dr_vel_e, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S);
            _blackout_dr_vel_d = 0.0f;

            _blackout_dr_n_m += static_cast<double>(_blackout_dr_vel_n) * dt_dr;
            _blackout_dr_e_m += static_cast<double>(_blackout_dr_vel_e) * dt_dr;

            constexpr float DR_VEL_DECAY_PER_STEP = 0.995f;
            _blackout_dr_vel_n *= DR_VEL_DECAY_PER_STEP;
            _blackout_dr_vel_e *= DR_VEL_DECAY_PER_STEP;

            if (fabsf(_blackout_dr_vel_n) < 0.02f) {
                _blackout_dr_vel_n = 0.0f;
            }

            if (fabsf(_blackout_dr_vel_e) < 0.02f) {
                _blackout_dr_vel_e = 0.0f;
            }

            if (!PX4_ISFINITE(_blackout_dr_alt)) {
                _blackout_dr_alt = _blackout_anchor_alt;
            }
        }

        const bool vio_alt_recent =
            _vio_valid &&
            (timestamp >= _vio_timestamp) &&
            ((timestamp - _vio_timestamp) <= BLACKOUT_VIO_TIMEOUT_US) &&
            PX4_ISFINITE(_vio_alt_msl);

        // A componente vertical usa VIO quando disponivel.
        // Sem VIO recente, preserva a altitude propagada pelo dead reckoning.
        if (!_gps_auto_disarm_sent) {
            double aux_alt_msl = _blackout_dr_alt;
            float aux_vel_d = _blackout_dr_vel_d;
            const char *aux_alt_src = "DR";

            if (vio_alt_recent) {
                aux_alt_msl = _vio_alt_msl;
                aux_vel_d = _vio_vel_d;
                aux_alt_src = "VIO";
            }

            if (PX4_ISFINITE(aux_alt_msl)) {
                if (_gps_landing_active) {
                    aux_alt_msl = math::max(_gps_land_target_alt_msl, aux_alt_msl);
                }

                _blackout_dr_alt = aux_alt_msl;
                _blackout_dr_vel_d = math::constrain(aux_vel_d, -JMIT_DR_VEL_Z_MAX_M_S, JMIT_DR_VEL_Z_MAX_M_S);

                static uint64_t blackout_alt_sync_log_last_us = 0;

                if ((timestamp - blackout_alt_sync_log_last_us) > 1000000ULL) {
                    blackout_alt_sync_log_last_us = timestamp;

                    PX4_INFO("\n[Blackout-Mitig] Alt sync | src=%s | alt=%.2f | vio_alt=%.2f\n",
                        aux_alt_src,
                        _blackout_dr_alt,
                        vio_alt_recent ? _vio_alt_msl : -1.0);
                }
            }
        }

        // Durante o pouso mitigado, mantem N/E fixos e so reduz altitude se pitch/roll estiverem baixos.
        if (_gps_landing_active) {
            const bool land_att_recent =
                _sim_att_valid &&
                (timestamp >= _sim_att_timestamp) &&
                ((timestamp - _sim_att_timestamp) <= 500000ULL);

            const float land_abs_pitch_rad = land_att_recent ?
                fabsf(_sim_pitch_rad) : 0.0f;

            const float land_abs_roll_rad = land_att_recent ?
                fabsf(_sim_roll_rad) : 0.0f;

            const bool descent_attitude_ok =
                !_gps_land_source_blackout ||
                !land_att_recent ||
                ((land_abs_pitch_rad <= AUTO_LAND_MAX_PITCH_RAD) &&
                (land_abs_roll_rad <= AUTO_LAND_MAX_ROLL_RAD));

            double land_alt_msl = _gps_land_last_alt_valid ?
                _gps_land_last_alt_msl : _gps_land_start_alt_msl;

            float land_vel_d = 0.0f;

            if (vio_alt_recent) {
                const double aux_land_alt_msl = _vio_alt_msl;

                if (PX4_ISFINITE(aux_land_alt_msl)) {
                    land_alt_msl = math::max(
                        _gps_land_target_alt_msl,
                        math::min(land_alt_msl, aux_land_alt_msl));
                }
            }

            const double dt_land_s =
                (_gps_land_last_update_us > 0 && timestamp >= _gps_land_last_update_us) ?
                math::constrain(static_cast<double>(timestamp - _gps_land_last_update_us) * 1e-6, 0.0, 0.1) :
                0.0;

            _gps_land_last_update_us = timestamp;

            if (descent_attitude_ok && !_gps_auto_disarm_sent) {
                land_alt_msl = math::max(
                    _gps_land_target_alt_msl,
                    land_alt_msl - (AUTO_LAND_DESCENT_RATE_M_S * dt_land_s));

                land_vel_d = (land_alt_msl <= _gps_land_target_alt_msl) ?
                    0.0f : static_cast<float>(AUTO_LAND_DESCENT_RATE_M_S);
            }

            if (_gps_auto_disarm_sent) {
                land_alt_msl = _gps_land_target_alt_msl;
                land_vel_d = 0.0f;
            }

            _gps_land_last_alt_msl = land_alt_msl;
            _gps_land_last_alt_valid = true;

            if (vio_recent) {
                _blackout_dr_n_m = _vio_n_m;
                _blackout_dr_e_m = _vio_e_m;
                _blackout_dr_vel_n = _vio_vel_n;
                _blackout_dr_vel_e = _vio_vel_e;

            } else if (lpos_recent) {
                _blackout_dr_n_m = _lpos_n_m;
                _blackout_dr_e_m = _lpos_e_m;
                _blackout_dr_vel_n = _imu_dr_vel_n;
                _blackout_dr_vel_e = _imu_dr_vel_e;

            } else {
                _blackout_dr_n_m = _gps_land_hold_n_m;
                _blackout_dr_e_m = _gps_land_hold_e_m;
                _blackout_dr_vel_n = 0.0f;
                _blackout_dr_vel_e = 0.0f;
            }

            _blackout_dr_alt = land_alt_msl;
            _blackout_dr_vel_d = descent_attitude_ok ? land_vel_d : 0.0f;

            static uint64_t pitch_hold_log_last_us = 0;

            if (!descent_attitude_ok &&
                    ((timestamp - pitch_hold_log_last_us) > AUTO_LAND_ATT_LOG_INTERVAL_US)) {
                pitch_hold_log_last_us = timestamp;

                PX4_INFO("\n[GPS-Land] Holding descent due attitude | pitch=%.1f deg | roll=%.1f deg | alt=%.2f\n",
                    land_att_recent ? static_cast<double>(_sim_pitch_rad * 57.2957795f) : -999.0,
                    land_att_recent ? static_cast<double>(_sim_roll_rad * 57.2957795f) : -999.0,
                    land_alt_msl);
            }
        }

        // Publica pseudo-GPS em 10 Hz durante a ausencia do GPS real.
        // A publicacao e interrompida apos o desarme.
        if (!_gps_auto_disarm_sent &&
                (timestamp - _blackout_last_pub_us) >= BLACKOUT_PUB_INTERVAL_US) {
            publishBlackoutPseudoGPS(timestamp);
            _blackout_last_pub_us = timestamp;
        }

    } else if (_blackout_anchor_initialized && gps_real_recent) {
        _blackout_anchor_initialized = false;
        _blackout_detected = false;
        PX4_INFO("\n[Blackout-Mitig] Real GPS recovered by timeout. Pseudo-GPS stopped.\n");
    }
    // ======== MITIGACAO BLACKOUT GPS - FIM ========

    // Diagnostico da inclinacao durante a aproximacao e o pouso.
    // Mostra se o roll/pitch aparece por correcao horizontal, velocidade residual ou divergencia entre fontes.
    if (_attack_mitigation_enabled &&
            _blackout_anchor_initialized &&
            _blackout_detected &&
            !_gps_auto_disarm_sent) {

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
                _blackout_dr_n_m - static_cast<double>(diag_dest_n_m);

            const double diag_de =
                _blackout_dr_e_m - static_cast<double>(diag_dest_e_m);

            diag_err_xy_m = sqrt(diag_dn * diag_dn + diag_de * diag_de);
        }

        const bool diag_near_landing =
            _gps_landing_active ||
            _gps_auto_land_sent ||
            (diag_err_xy_m >= 0.0 && diag_err_xy_m <= 3.0) ||
            (diag_agl_recent && _sim_agl_m <= 3.0);

        if (diag_near_landing &&
                ((timestamp - tilt_diag_last_us) > 250000ULL)) {
            tilt_diag_last_us = timestamp;

            const char *diag_phase = _gps_landing_active ?
                "LAND_ACTIVE" : (_gps_auto_land_sent ? "LAND_SENT" : "APPROACH");

            const double diag_roll_deg = diag_att_recent ?
                static_cast<double>(_sim_roll_rad * 57.2957795f) : -999.0;

            const double diag_pitch_deg = diag_att_recent ?
                static_cast<double>(_sim_pitch_rad * 57.2957795f) : -999.0;

            const double diag_agl_m = diag_agl_recent ? _sim_agl_m : -1.0;

            const double diag_lpos_dn =
                diag_lpos_recent ? (_lpos_n_m - _blackout_dr_n_m) : 999.0;

            const double diag_lpos_de =
                diag_lpos_recent ? (_lpos_e_m - _blackout_dr_e_m) : 999.0;

            const double diag_vio_dn =
                diag_vio_recent ? (_vio_n_m - _blackout_dr_n_m) : 999.0;

            const double diag_vio_de =
                diag_vio_recent ? (_vio_e_m - _blackout_dr_e_m) : 999.0;

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
                static_cast<double>(_blackout_dr_vel_n),
                static_cast<double>(_blackout_dr_vel_e),
                static_cast<double>(_blackout_dr_vel_d),
                diag_lpos_dn,
                diag_lpos_de,
                diag_vio_dn,
                diag_vio_de,
                _blackout_dr_n_m,
                _blackout_dr_e_m,
                _blackout_dr_alt,
                diag_lpos_recent ? _lpos_n_m : 999.0,
                diag_lpos_recent ? _lpos_e_m : 999.0,
                diag_lpos_recent ? _lpos_alt_msl : 999.0,
                diag_vio_recent ? _vio_n_m : 999.0,
                diag_vio_recent ? _vio_e_m : 999.0,
                diag_vio_recent ? _vio_alt_msl : 999.0);
        }
    }

    // ======== MITIGACAO DE POUSO GPS - INICIO ========
    checkGpsAutoLand(timestamp);
    checkGpsAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO GPS - FIM ========

    // ======== MITIGACAO DE POUSO IMU - INICIO ========
    checkImuAutoLand(timestamp);
    checkImuAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO IMU - FIM ========

    // ======== MITIGACAO DE POUSO MOTOR - INICIO ========
    checkMotorAutoLand(timestamp);
    checkMotorAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO MOTOR - FIM ========

    // ======== MITIGACAO DE POUSO MAGNETOMETRO - INICIO ========
    checkMagAutoLand(timestamp);
    checkMagAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO MAGNETOMETRO - FIM ========

    // ======== MITIGACAO DE POUSO LIDAR - INICIO ========
    checkLidarAutoLand(timestamp);
    checkLidarAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO LIDAR - FIM ========

    // ======== MITIGACAO DE POUSO LIDAR FRONTAL - INICIO ========
    checkLidarFrontAutoLand(timestamp);
    checkLidarFrontAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO LIDAR FRONTAL - FIM ========

    // ======== MITIGACAO DE POUSO LIDAR 2D - INICIO ========
    checkLidar2dAutoLand(timestamp);
    checkLidar2dAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO LIDAR 2D - FIM ========

    // ======== MITIGACAO DE POUSO BAROMETRO - INICIO ========
    checkBaroAutoLand(timestamp);
    checkBaroAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO BAROMETRO - FIM ========

}

// ======== MITIGACAO DE POUSO GPS - INICIO ========
// Controla o pouso quando a mitigacao GPS esta ativa.
// A origem pode ser blackout ou anomalia GPS persistente rejeitada pelo filtro.
void GZBridge::checkGpsAutoLand(uint64_t timestamp)
{
    if (!_attack_mitigation_enabled || _gps_auto_land_sent || !_pos_ref.isInitialized()) {
        _gps_auto_land_arrival_us = 0;
        _gps_auto_land_last_arrived_us = 0;
        return;
    }

    const bool blackout_active = _blackout_anchor_initialized && _blackout_detected;
    const bool gps_anomaly_active = _kf_initialized && (_kf_attack_dur_us >= AUTO_LAND_MIN_NOISE_US);

    bool use_blackout_source = false;

    if (blackout_active) {
        if ((timestamp - _blackout_start_us) < AUTO_LAND_MIN_BLACKOUT_US) {
            return;
        }

        use_blackout_source = true;

    } else if (gps_anomaly_active) {
        use_blackout_source = false;

    } else {
        const bool keep_arrival_timer =
            (_gps_auto_land_arrival_us > 0) &&
            (_gps_auto_land_last_arrived_us > 0) &&
            (timestamp >= _gps_auto_land_last_arrived_us) &&
            ((timestamp - _gps_auto_land_last_arrived_us) <= AUTO_LAND_PULSE_BRIDGE_US);

        if (!keep_arrival_timer) {
            _gps_auto_land_arrival_us = 0;
            _gps_auto_land_last_arrived_us = 0;
        }

        return;
    }

    if (!_lpos_xy_valid || (timestamp - _lpos_timestamp) > 500000ULL) {
        _gps_auto_land_arrival_us = 0;
        _gps_auto_land_last_arrived_us = 0;
        return;
    }

    position_setpoint_triplet_s triplet{};

    if (!_pos_sp_triplet_sub.copy(&triplet) || !triplet.current.valid ||
            !PX4_ISFINITE(triplet.current.lat) || !PX4_ISFINITE(triplet.current.lon)) {
        _gps_auto_land_arrival_us = 0;
        _gps_auto_land_last_arrived_us = 0;
        return;
    }

    float dest_n_m = 0.0f;
    float dest_e_m = 0.0f;
    _pos_ref.project(triplet.current.lat, triplet.current.lon, dest_n_m, dest_e_m);

    const bool severe_anomaly_recent =
        !use_blackout_source &&
        _gps_anomaly_severe &&
        (_kf_attack_dur_us >= AUTO_LAND_MIN_NOISE_US);

    double land_est_n_m = _lpos_n_m;
    double land_est_e_m = _lpos_e_m;
    float land_est_speed_m_s = _lpos_ground_speed;

    if (use_blackout_source) {
        land_est_n_m = _blackout_dr_n_m;
        land_est_e_m = _blackout_dr_e_m;
        land_est_speed_m_s = sqrtf(
            _blackout_dr_vel_n * _blackout_dr_vel_n +
            _blackout_dr_vel_e * _blackout_dr_vel_e);

    } else if (_kf_attack_dur_us > 0 &&
            PX4_ISFINITE(_jmit_pub_n_m) && PX4_ISFINITE(_jmit_pub_e_m)) {
        land_est_n_m = _jmit_pub_n_m;
        land_est_e_m = _jmit_pub_e_m;
        land_est_speed_m_s = sqrtf(
            _jmit_pub_vel_n * _jmit_pub_vel_n +
            _jmit_pub_vel_e * _jmit_pub_vel_e);
    }

    const double dn = land_est_n_m - static_cast<double>(dest_n_m);
    const double de = land_est_e_m - static_cast<double>(dest_e_m);
    const float horizontal_error = static_cast<float>(sqrt(dn * dn + de * de));

    const float land_radius_m = use_blackout_source ?
        AUTO_LAND_DEST_RADIUS_M :
        (severe_anomaly_recent ?
            AUTO_LAND_SEVERE_ANOMALY_DEST_RADIUS_M :
            AUTO_LAND_ANOMALY_DEST_RADIUS_M);

    const float land_max_speed_m_s = use_blackout_source ?
        AUTO_LAND_MAX_SPEED_M_S :
        (severe_anomaly_recent ?
            AUTO_LAND_SEVERE_ANOMALY_MAX_SPEED_M_S :
            AUTO_LAND_ANOMALY_MAX_SPEED_M_S);

    const bool arrived =
        horizontal_error <= land_radius_m &&
        land_est_speed_m_s <= land_max_speed_m_s;

    const uint64_t stable_required_us = use_blackout_source ?
        AUTO_LAND_STABLE_US :
        (severe_anomaly_recent ? AUTO_LAND_SEVERE_STABLE_US : AUTO_LAND_STABLE_US);

    if (!arrived) {
        _gps_auto_land_arrival_us = 0;
        _gps_auto_land_last_arrived_us = 0;
        return;
    }

    _gps_auto_land_last_arrived_us = timestamp;

    const bool preland_att_recent =
        _sim_att_valid &&
        (timestamp >= _sim_att_timestamp) &&
        ((timestamp - _sim_att_timestamp) <= 500000ULL);

    const float preland_abs_pitch_rad = preland_att_recent ?
        fabsf(_sim_pitch_rad) : 0.0f;

    const float preland_abs_roll_rad = preland_att_recent ?
        fabsf(_sim_roll_rad) : 0.0f;

    const bool preland_tilt_high =
        use_blackout_source &&
        preland_att_recent &&
        ((preland_abs_pitch_rad > AUTO_LAND_MAX_PITCH_RAD) ||
        (preland_abs_roll_rad > AUTO_LAND_MAX_ROLL_RAD));

    if (preland_tilt_high) {
        static uint64_t preland_tilt_log_last_us = 0;

        if ((timestamp - preland_tilt_log_last_us) > AUTO_LAND_ATT_LOG_INTERVAL_US) {
            preland_tilt_log_last_us = timestamp;

            PX4_WARN("\n[GPS-Land] Tilt observed before LAND, but not blocking descent | pitch=%.1f deg | roll=%.1f deg | err_xy=%.1f m | vel=%.2f m/s\n",
                static_cast<double>(_sim_pitch_rad * 57.2957795f),
                static_cast<double>(_sim_roll_rad * 57.2957795f),
                static_cast<double>(horizontal_error),
                static_cast<double>(land_est_speed_m_s));
        }
    }

    if (_gps_auto_land_arrival_us == 0) {
        _gps_auto_land_arrival_us = timestamp;
        _gps_land_source_blackout = use_blackout_source;
        _gps_land_source_severe_anomaly = severe_anomaly_recent;
        _gps_land_hold_n_m = land_est_n_m;
        _gps_land_hold_e_m = land_est_e_m;

        // Mantem o heading local disponivel para o comando LAND.
        _gps_land_heading_rad = _lpos_heading_rad;
        _gps_land_heading_valid = PX4_ISFINITE(_lpos_heading_rad);

        if (!use_blackout_source) {
            if (PX4_ISFINITE(_jmit_pub_alt) && _kf_attack_dur_us > 0) {
                _gps_land_start_alt_msl = _jmit_pub_alt;
            } else if (PX4_ISFINITE(_kf_alt)) {
                _gps_land_start_alt_msl = _kf_alt;
            } else {
                _gps_land_start_alt_msl = _gps_real_alt;
            }

            _gps_land_target_alt_msl = static_cast<double>(_alt_ref) + AUTO_LAND_TARGET_GPS_ANOMALY_AGL_M;

            if (_gps_land_start_alt_msl < _gps_land_target_alt_msl) {
                _gps_land_start_alt_msl = _gps_land_target_alt_msl;
            }

            _gps_land_last_alt_msl = _gps_land_start_alt_msl;
            _gps_land_last_alt_valid = true;
        }

        PX4_INFO("\n[GPS-Land] Destination reached | src=%s | err_xy=%.1f m | vel=%.2f m/s | window=%.1f s\n",
            use_blackout_source ? "blackout" : "anomaly",
            static_cast<double>(horizontal_error),
            static_cast<double>(land_est_speed_m_s),
            static_cast<double>(stable_required_us) / 1e6);

        if (stable_required_us > 0ULL) {
            return;
        }
    }

    const uint64_t preland_elapsed_us = timestamp - _gps_auto_land_arrival_us;

    if (preland_elapsed_us < stable_required_us) {
        return;
    }

    const bool final_speed_check_needed =
        !use_blackout_source &&
        severe_anomaly_recent;

    if (final_speed_check_needed) {
        const bool lpos_speed_recent =
            _lpos_xy_valid &&
            (timestamp >= _lpos_timestamp) &&
            ((timestamp - _lpos_timestamp) <= 500000ULL) &&
            PX4_ISFINITE(_lpos_ground_speed);

        const float final_speed_m_s = lpos_speed_recent ?
            _lpos_ground_speed : land_est_speed_m_s;

        const bool final_wait_timeout =
            preland_elapsed_us >= AUTO_LAND_FINAL_MAX_WAIT_US;

        if ((final_speed_m_s > AUTO_LAND_FINAL_MAX_SPEED_M_S) &&
                !final_wait_timeout) {
            return;
        }
    }

    _gps_landing_active = true;
    _gps_auto_land_sent = true;
    _gps_land_source_blackout = use_blackout_source;
    _gps_land_source_severe_anomaly = (!use_blackout_source) && (_gps_land_source_severe_anomaly || severe_anomaly_recent);
    _gps_land_hold_start_us = timestamp;

    if (use_blackout_source) {
        _gps_land_hold_n_m = land_est_n_m;
        _gps_land_hold_e_m = land_est_e_m;
    }

    if (use_blackout_source) {
        _gps_land_start_alt_msl = PX4_ISFINITE(_blackout_dr_alt) ? _blackout_dr_alt : _blackout_anchor_alt;
        _gps_land_target_alt_msl = static_cast<double>(_alt_ref) + AUTO_LAND_TARGET_AGL_M;
    } else {
        if (_gps_land_last_alt_valid && PX4_ISFINITE(_gps_land_last_alt_msl)) {
            _gps_land_start_alt_msl = _gps_land_last_alt_msl;
        } else if (PX4_ISFINITE(_jmit_pub_alt) && _kf_attack_dur_us > 0) {
            _gps_land_start_alt_msl = _jmit_pub_alt;
        } else if (PX4_ISFINITE(_kf_alt)) {
            _gps_land_start_alt_msl = _kf_alt;
        } else {
            _gps_land_start_alt_msl = _gps_real_alt;
        }

        _gps_land_target_alt_msl = static_cast<double>(_alt_ref) + AUTO_LAND_TARGET_GPS_ANOMALY_AGL_M;
    }

    if (_gps_land_start_alt_msl < _gps_land_target_alt_msl) {
        _gps_land_start_alt_msl = _gps_land_target_alt_msl;
    }

    const float initial_land_vel_d = use_blackout_source ?
        static_cast<float>(AUTO_LAND_DESCENT_RATE_M_S) : 0.0f;

    _jmit_pub_n_m = _gps_land_hold_n_m;
    _jmit_pub_e_m = _gps_land_hold_e_m;
    _jmit_pub_alt = _gps_land_start_alt_msl;
    _jmit_pub_vel_n = 0.0f;
    _jmit_pub_vel_e = 0.0f;
    _jmit_pub_vel_d = initial_land_vel_d;

    _kf_n_m = _jmit_pub_n_m;
    _kf_e_m = _jmit_pub_e_m;
    _kf_alt = _jmit_pub_alt;
    _kf_vel_n = 0.0f;
    _kf_vel_e = 0.0f;
    _kf_vel_d = initial_land_vel_d;

    _blackout_dr_n_m = _gps_land_hold_n_m;
    _blackout_dr_e_m = _gps_land_hold_e_m;
    _blackout_dr_alt = _gps_land_start_alt_msl;
    _blackout_dr_vel_n = 0.0f;
    _blackout_dr_vel_e = 0.0f;
    _blackout_dr_vel_d = initial_land_vel_d;

    _gps_land_last_alt_msl = _gps_land_start_alt_msl;
    _gps_land_last_alt_valid = true;
    _gps_land_last_update_us = timestamp;

    publishGpsLandCommand(timestamp);
}

// Envia comando de desarme assim que o detector de pouso do PX4 confirmar contato com o solo.
// Isso evita cortar os motores durante a descida quando a altitude estimada ainda e imprecisa.
void GZBridge::checkGpsAutoDisarm(uint64_t timestamp)
{
    if (!_gps_landing_active || !_gps_auto_land_sent || _gps_auto_disarm_sent) {
        return;
    }

    // No blackout, o pouso pode tocar o solo antes da janela minima anterior.
    // Assim que o land detector ou o sensor inferior indicar contato, o desarme e liberado.
    vehicle_land_detected_s land_detected{};
    const bool land_detector_available = _vehicle_land_detected_sub.copy(&land_detected);

    const bool land_detector_recent =
        land_detector_available &&
        (land_detected.timestamp > 0) &&
        (timestamp >= land_detected.timestamp) &&
        ((timestamp - land_detected.timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US);

    const bool ground_contact_by_px4_landed =
        land_detector_recent &&
        land_detected.landed;

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

    const bool sim_ground_recent =
        _sim_agl_valid &&
        (timestamp >= _sim_agl_timestamp) &&
        ((timestamp - _sim_agl_timestamp) <= AUTO_LAND_GROUND_SENSOR_TIMEOUT_US) &&
        PX4_ISFINITE(_sim_agl_m);

    // Atualiza o contador de histerese: incrementa apenas quando o AGL esta abaixo do limiar
    // em amostras consecutivas, resetando ao ultrapassar o limiar. Isso evita o desarme
    // prematuro quando o drone toca o solo inclinado e oscila em torno do limiar.
    if (sim_ground_recent) {
        if (_sim_agl_m <= AUTO_LAND_SIM_GROUND_DISARM_AGL_M) {
            _sim_agl_ground_count++;
        } else {
            _sim_agl_ground_count = 0;
        }
    } else {
        _sim_agl_ground_count = 0;
    }

    const bool ground_contact_by_sim_agl =
        sim_ground_recent &&
        (_sim_agl_m <= AUTO_LAND_SIM_GROUND_DISARM_AGL_M) &&
        (_sim_agl_ground_count >= SIM_AGL_GROUND_CONFIRM_COUNT);

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

    const double disarm_roll_deg = disarm_att_recent ?
        static_cast<double>(_sim_roll_rad * 57.2957795f) : -999.0;

    const double disarm_pitch_deg = disarm_att_recent ?
        static_cast<double>(_sim_pitch_rad * 57.2957795f) : -999.0;

    const float disarm_jmit_gs = sqrtf(
        _jmit_pub_vel_n * _jmit_pub_vel_n +
        _jmit_pub_vel_e * _jmit_pub_vel_e);

    const bool forced_disarm_speed_ok =
        (_lpos_ground_speed <= 0.15f) &&
        (disarm_jmit_gs <= 0.10f);

    const bool forced_disarm_att_ok =
        !disarm_att_recent ||
        ((fabsf(_sim_roll_rad) <= AUTO_LAND_MAX_ROLL_RAD) &&
        (fabsf(_sim_pitch_rad) <= AUTO_LAND_MAX_PITCH_RAD));

    if (force_disarm && (!forced_disarm_speed_ok || !forced_disarm_att_ok)) {
        static uint64_t disarm_wait_log_last_us = 0;

        if ((timestamp - disarm_wait_log_last_us) > AUTO_LAND_ATT_LOG_INTERVAL_US) {
            disarm_wait_log_last_us = timestamp;

            PX4_INFO("\n[GPS-Land] Waiting before forced disarm | agl=%.2f | dist=%.2f | lpos_gs=%.2f | jmit_gs=%.2f | roll=%.1f pitch=%.1f\n",
                sim_ground_recent ? static_cast<double>(_sim_agl_m) : -1.0,
                ground_sensor_recent ? static_cast<double>(_ground_distance_m) : -1.0,
                static_cast<double>(_lpos_ground_speed),
                static_cast<double>(disarm_jmit_gs),
                disarm_roll_deg,
                disarm_pitch_deg);
        }

        return;
    }

    if (ground_contact_by_px4_landed) {
        PX4_WARN("\n[GPS-Land] PX4 landing detector confirmed landed | roll=%.1f pitch=%.1f - normal disarm\n",
            disarm_roll_deg,
            disarm_pitch_deg);
    } else if (ground_contact_by_px4_early) {
        PX4_WARN("\n[GPS-Land] PX4 landing detector reported ground contact | roll=%.1f pitch=%.1f - safety disarm\n",
            disarm_roll_deg,
            disarm_pitch_deg);
    } else if (ground_contact_by_sensor) {
        PX4_WARN("\n[GPS-Land] Downward distance sensor reported ground contact | dist=%.2f m | roll=%.1f pitch=%.1f - safety disarm\n",
            static_cast<double>(_ground_distance_m),
            disarm_roll_deg,
            disarm_pitch_deg);
    } else {
        PX4_WARN("\n[GPS-Land] Simulated ground proximity reached | agl=%.2f m | roll=%.1f pitch=%.1f - safety disarm\n",
            static_cast<double>(_sim_agl_m),
            disarm_roll_deg,
            disarm_pitch_deg);
    }

    publishGpsDisarmCommand(timestamp, force_disarm);
    _gps_auto_disarm_sent = true;
    _gps_auto_land_sent = true;
    _gps_landing_active = false;
}

// Envia LAND com coordenada global explicita no ponto de chegada.
void GZBridge::publishGpsLandCommand(uint64_t timestamp)
{
    double land_lat = 0.0;
    double land_lon = 0.0;
    _pos_ref.reproject(
        static_cast<float>(_gps_land_hold_n_m),
        static_cast<float>(_gps_land_hold_e_m),
        land_lat,
        land_lon);

    vehicle_command_s cmd{};
    cmd.timestamp = timestamp;
    cmd.param1 = 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;

    cmd.param4 = NAN;

    if (_gps_land_source_blackout) {
        // Blackout precisa de coordenada/altitude explicita porque o GPS real foi interrompido.
        cmd.param5 = static_cast<float>(land_lat);
        cmd.param6 = static_cast<float>(land_lon);
        cmd.param7 = static_cast<float>(_gps_land_target_alt_msl);
    } else if (_gps_land_source_severe_anomaly) {
        // Anomalia severa: fixa apenas a posicao horizontal.
        // A descida vertical fica com o controlador LAND do PX4.
        cmd.param5 = static_cast<float>(land_lat);
        cmd.param6 = static_cast<float>(land_lon);
        cmd.param7 = NAN;
    } else {
        // Anomalia moderada: fixa as coordenadas horizontais e deixa a altitude sob controle do modo LAND.
        // Isso evita deriva da missao e preserva a descida fisica controlada pelo PX4.
        cmd.param5 = static_cast<float>(land_lat);
        cmd.param6 = static_cast<float>(land_lon);
        cmd.param7 = NAN;
    }

    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 0;
    cmd.from_external = false;

    _vehicle_command_pub.publish(cmd);

    PX4_WARN("\n[GPS-Land] Controlled landing started | src=%s | hold N=%.1f E=%.1f | alt %.1f -> %s | lat=%.7f lon=%.7f\n",
        _gps_land_source_blackout ? "blackout" : "anomaly",
        _gps_land_hold_n_m,
        _gps_land_hold_e_m,
        _gps_land_start_alt_msl,
        _gps_land_source_blackout ? "target" : "fixed-pos",
        land_lat,
        land_lon);
}

// Envia o comando de desarme do PX4 apos confirmacao de contato com o solo.
void GZBridge::publishGpsDisarmCommand(uint64_t timestamp, bool force_disarm)
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

    PX4_WARN("\n[GPS-Land] %s disarm command sent after ground contact confirmation.\n",
        force_disarm ? "Forced" : "Normal");
}
// ======== MITIGACAO DE POUSO GPS - FIM ========

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
void GZBridge::checkImuAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _imu_anomaly_active, _imu_landing_active,
        _imu_auto_land_sent, _imu_auto_land_arrival_us, _imu_auto_land_last_arrived_us,
        _imu_land_hold_n_m, _imu_land_hold_e_m, "IMU");
}

void GZBridge::checkImuAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _imu_landing_active, _imu_auto_land_sent,
        _imu_auto_disarm_sent, _imu_sim_agl_ground_count, "IMU");
}
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
void GZBridge::checkMagAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _mag_anomaly_active, _mag_landing_active,
        _mag_auto_land_sent, _mag_auto_land_arrival_us, _mag_auto_land_last_arrived_us,
        _mag_land_hold_n_m, _mag_land_hold_e_m, "Mag");
}

void GZBridge::checkMagAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _mag_landing_active, _mag_auto_land_sent,
        _mag_auto_disarm_sent, _mag_sim_agl_ground_count, "Mag");
}
// ======== MITIGACAO DE POUSO MAGNETOMETRO - FIM ========

// ======== MITIGACAO DE POUSO LIDAR - INICIO ========
void GZBridge::checkLidarAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _lidar_anomaly_active, _lidar_landing_active,
        _lidar_auto_land_sent, _lidar_auto_land_arrival_us, _lidar_auto_land_last_arrived_us,
        _lidar_land_hold_n_m, _lidar_land_hold_e_m, "Lidar");
}

void GZBridge::checkLidarAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _lidar_landing_active, _lidar_auto_land_sent,
        _lidar_auto_disarm_sent, _lidar_sim_agl_ground_count, "Lidar");
}
// ======== MITIGACAO DE POUSO LIDAR - FIM ========

// ======== MITIGACAO DE POUSO LIDAR FRONTAL - INICIO ========
void GZBridge::checkLidarFrontAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _lidar_front_anomaly_active, _lidar_front_landing_active,
        _lidar_front_auto_land_sent, _lidar_front_auto_land_arrival_us, _lidar_front_auto_land_last_arrived_us,
        _lidar_front_land_hold_n_m, _lidar_front_land_hold_e_m, "LidarFront");
}

void GZBridge::checkLidarFrontAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _lidar_front_landing_active, _lidar_front_auto_land_sent,
        _lidar_front_auto_disarm_sent, _lidar_front_sim_agl_ground_count, "LidarFront");
}
// ======== MITIGACAO DE POUSO LIDAR FRONTAL - FIM ========

// ======== MITIGACAO DE POUSO LIDAR 2D - INICIO ========
void GZBridge::checkLidar2dAutoLand(uint64_t timestamp)
{
    checkGenericAnomalyAutoLand(timestamp, _lidar2d_anomaly_active, _lidar2d_landing_active,
        _lidar2d_auto_land_sent, _lidar2d_auto_land_arrival_us, _lidar2d_auto_land_last_arrived_us,
        _lidar2d_land_hold_n_m, _lidar2d_land_hold_e_m, "Lidar2D");
}

void GZBridge::checkLidar2dAutoDisarm(uint64_t timestamp)
{
    checkGenericAnomalyAutoDisarm(timestamp, _lidar2d_landing_active, _lidar2d_auto_land_sent,
        _lidar2d_auto_disarm_sent, _lidar2d_sim_agl_ground_count, "Lidar2D");
}
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


// ======== MITIGACAO BLACKOUT GPS - INICIO ========
// Publica uma pseudo-medicao GPS durante blackout usando VIO quando disponivel
// ou dead-reckoning horizontal a partir da ultima velocidade confiavel.
void GZBridge::publishBlackoutPseudoGPS(uint64_t timestamp)
{
    if (!_blackout_anchor_initialized || !_pos_ref.isInitialized()) {
        return;
    }

    // Limite de seguranca: evita manter pseudo-GPS por tempo indefinido.
    const uint64_t blackout_elapsed_us = timestamp - _blackout_start_us;

    if (blackout_elapsed_us > BLACKOUT_MAX_HOLD_US) {
        PX4_WARN("\n[Blackout-Mitig] Maximum dead-reckoning time reached. Pseudo-GPS stopped.\n");
        return;
    }

    double latitude = 0.0;
    double longitude = 0.0;

    // Converte a posicao local estimada N/E de volta para latitude/longitude.
    _pos_ref.reproject(
        static_cast<float>(_blackout_dr_n_m),
        static_cast<float>(_blackout_dr_e_m),
        latitude,
        longitude
    );

    device::Device::DeviceId id{};
    id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
    id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
    id.devid_s.bus = 1;
    id.devid_s.address = 1;

    sensor_gps_s sensor_gps{};

    sensor_gps.timestamp = timestamp;
    sensor_gps.timestamp_sample = timestamp;
    sensor_gps.device_id = id.devid;

    sensor_gps.latitude_deg = latitude;
    sensor_gps.longitude_deg = longitude;
    sensor_gps.altitude_msl_m = _blackout_dr_alt;
    sensor_gps.altitude_ellipsoid_m = _blackout_dr_alt;

    sensor_gps.vel_n_m_s = _blackout_dr_vel_n;
    sensor_gps.vel_e_m_s = _blackout_dr_vel_e;
    sensor_gps.vel_d_m_s = _blackout_dr_vel_d;
    sensor_gps.vel_m_s = sqrtf(
        _blackout_dr_vel_n * _blackout_dr_vel_n +
        _blackout_dr_vel_e * _blackout_dr_vel_e
    );

    // Durante o pouso ativo, usa o heading capturado como COG da pseudo-GPS.
    // Com velocidade zero, atan2(0,0) retorna 0 independente do heading real,
    // o que pode causar salto de estimativa no EKF. Usar o heading do drone
    // mantem a estimativa consistente e evita o pouso inclinado.
    if (_gps_landing_active && _gps_land_heading_valid) {
        sensor_gps.cog_rad = _gps_land_heading_rad;
    } else {
        sensor_gps.cog_rad = atan2f(_blackout_dr_vel_e, _blackout_dr_vel_n);
    }
    sensor_gps.vel_ned_valid = true;

    // Mantem fix valido com qualidade degradada para representar GPS mitigado em contingencia.
    sensor_gps.fix_type = 3;
    sensor_gps.satellites_used = math::max(static_cast<int>(_sim_gps_used.get()), 4);

    // Durante o pouso ativo, eleva a qualidade da pseudo-GPS para coincidir com a
    // publicacao da navSatCallback e evitar inconsistencia no EKF durante a descida.
    if (_gps_landing_active) {
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

    // A pseudo-medicao mitigada e tratada como saida operacional da contramedida.

    sensor_gps.jamming_indicator = 0;
    sensor_gps.jamming_state = 0;

    _sensor_gps_pub.publish(sensor_gps);

    static uint64_t blackout_log_last_us = 0;

    if ((timestamp - blackout_log_last_us) > 1000000ULL) {
        blackout_log_last_us = timestamp;

        const bool vio_recent =
            _vio_valid &&
            ((timestamp - _vio_timestamp) <= BLACKOUT_VIO_TIMEOUT_US);

        PX4_INFO("\n[Blackout-Mitig] Pseudo-GPS | src=%s | t=%.1f s | N=%.1f E=%.1f Alt=%.1f | Vel=(%.1f, %.1f, %.1f)\n",
            vio_recent ? "VIO" : "DR-HOR",
            static_cast<double>(blackout_elapsed_us) / 1e6,
            _blackout_dr_n_m,
            _blackout_dr_e_m,
            _blackout_dr_alt,
            static_cast<double>(_blackout_dr_vel_n),
            static_cast<double>(_blackout_dr_vel_e),
            static_cast<double>(_blackout_dr_vel_d));
    }
}
// ======== MITIGACAO BLACKOUT GPS - FIM ========

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
    // cada atualizacao. Consumido em imuCallback quando uma anomalia esta ativa.
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
                _vio_imu_accel_body[0] = specific_force_body(0);
                _vio_imu_accel_body[1] = specific_force_body(1);
                _vio_imu_accel_body[2] = specific_force_body(2);
                _vio_imu_accel_valid = true;
                _vio_imu_accel_us = timestamp;
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
            _vio_imu_gyro_body[0] = vio_gyro_x;
            _vio_imu_gyro_body[1] = vio_gyro_y;
            _vio_imu_gyro_body[2] = vio_gyro_z;
            _vio_imu_gyro_valid = true;
            _vio_imu_gyro_us = timestamp;
        }
    }
    // ======== MITIGACAO IMU - FIM ========

    // ======== MITIGACAO MAGNETOMETRO - INICIO ========
    // Extrai heading, roll e pitch da atitude da propria VIO, referencia
    // independente do magnetometro usada em magnetometerCallback.
    if (vio_sample_accepted) {
        const matrix::Quatf q_nb_att(q_nb.W(), q_nb.X(), q_nb.Y(), q_nb.Z());
        const matrix::Eulerf euler_vio{q_nb_att};

        if (PX4_ISFINITE(euler_vio.psi()) && PX4_ISFINITE(euler_vio.phi()) && PX4_ISFINITE(euler_vio.theta())) {
            _vio_heading_rad = euler_vio.psi();
            _vio_roll_rad = euler_vio.phi();
            _vio_pitch_rad = euler_vio.theta();
            _vio_heading_valid = true;
            _vio_heading_us = timestamp;
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

    // ======== ATAQUE 2 GPS BLACKOUT - INICIO ========
    // Ataque 2: blackout GPS. O ataque apenas bloqueia a publicacao do GPS real.
    // A mitigacao detecta a falha por timeout
    // da ultima publicacao GPS real no imuCallback().
    // if (_jamming_attack_type == 2) {
    if (_jamming_attack.isBlackoutActive()) {

        // Garante que a origem de projecao local exista para permitir reprojecao do pseudo-GPS.
        if (!_pos_ref.isInitialized()) {
            _pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
            _alt_ref = msg.altitude();
        }

        return;
    }
    // ======== ATAQUE 2 GPS BLACKOUT - FIM ========

    // Apos o desarme, interrompe a publicacao de medicoes GPS no chao.
    if (_gps_auto_disarm_sent) {
        return;
    }

    // A primeira amostra define a origem local usada para projecao/reprojecao GPS.
    if (!_pos_ref.isInitialized()) {
        _pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
        _alt_ref = msg.altitude();
        return;
    }

    // ======== ATAQUE GPS OFFSET E ROTACAO - INICIO ========
    // Ataques GPS por offset e rotacao sao aplicados antes do ruido nominal.
    // double lat_offset = _gps_attack_offset.X();
    // double lon_offset = _gps_attack_offset.Y();
    double lat_offset = _gps_offset_attack.offsetLatDeg();
    double lon_offset = _gps_offset_attack.offsetLonDeg();

    // if (fabs(_gps_attack_rot.X()) > 1e-6 || fabs(_gps_attack_rot.Y()) > 1e-6) {
        // double angle_rad = math::radians(_gps_attack_rot.Y());
        // double radius_deg = _gps_attack_rot.X();
        // lat_offset += radius_deg * cos(angle_rad);
        // lon_offset += radius_deg * sin(angle_rad);
    // }
    if (fabs(_gps_rot_attack.radiusDeg()) > 1e-6 || fabs(_gps_rot_attack.angleDeg()) > 1e-6) {
        double angle_rad = math::radians(_gps_rot_attack.angleDeg());
        double radius_deg = _gps_rot_attack.radiusDeg();
        lat_offset += radius_deg * cos(angle_rad);
        lon_offset += radius_deg * sin(angle_rad);
    }

    double latitude = msg.latitude_deg() + lat_offset;
    double longitude = msg.longitude_deg() + lon_offset;
    // double altitude = msg.altitude() + _gps_attack_offset.Z() + _gps_attack_rot.Z();
    double altitude = msg.altitude() + _gps_offset_attack.offsetAltM() + _gps_rot_attack.offsetAltM();
    // ======== ATAQUE GPS OFFSET E ROTACAO - FIM ========

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

    // ======== ATAQUE 1 GPS NOISE JAMMING - INICIO ========
    // Ataque 1: jamming por ruido continuo. A perturbacao afeta posicao, altitude e velocidade.
    // if (_jamming_attack_type == 1) {

        // Ruido horizontal em metros convertido para latitude/longitude.
        // const double lat_rad = math::radians(latitude);
        // const double noise_n_m = static_cast<double>(generate_wgn() * _jamming_intensity * 10.0f);
        // const double noise_e_m = static_cast<double>(generate_wgn() * _jamming_intensity * 10.0f);

        // latitude  += math::degrees(noise_n_m / CONSTANTS_RADIUS_OF_EARTH);
        // longitude += math::degrees(noise_e_m / (CONSTANTS_RADIUS_OF_EARTH * cos(lat_rad)));

        // O canal vertical recebe ruido maior para representar a menor precisao vertical do GPS.
        // altitude += static_cast<double>(generate_wgn() * _jamming_intensity * 30.0f);

        // Velocidades tambem sao perturbadas para simular uma medicao GPS dinamicamente inconsistente.
        // vel_north += generate_wgn() * _jamming_intensity * 10.0f;
        // vel_east += generate_wgn() * _jamming_intensity * 10.0f;
        // vel_down += generate_wgn() * _jamming_intensity * 10.0f;
    // }
    _jamming_attack.applyNoiseJamming(latitude, longitude, altitude, vel_north, vel_east, vel_down);
    // ======== ATAQUE 1 GPS NOISE JAMMING - FIM ========

    // ======== MITIGACAO GPS - INICIO ========
    bool gps_anomalous_now = false;
    // Mitigacao GPS: detecta medicoes anomalas por NIS e publica uma pseudo-medicao degradada.
    {
        const uint64_t now = hrt_absolute_time();

        // Projecao da medicao GPS atual para coordenadas locais em metros.
        float cur_n_m, cur_e_m;
        _pos_ref.project(latitude, longitude, cur_n_m, cur_e_m);
        const double meas_n   = static_cast<double>(cur_n_m);
        const double meas_e   = static_cast<double>(cur_e_m);
        const double meas_alt = altitude;

        const bool lpos_xy_recent =
            _lpos_xy_valid &&
            (now >= _lpos_timestamp) &&
            ((now - _lpos_timestamp) <= 500000ULL);

        const bool lpos_alt_recent =
            _jmit_lpos_alt_valid &&
            (now >= _jmit_lpos_timestamp) &&
            ((now - _jmit_lpos_timestamp) <= 500000ULL) &&
            PX4_ISFINITE(_jmit_lpos_alt_msl);

        const bool baro_alt_recent =
            _jmit_baro_valid &&
            _jmit_baro_alt_offset_valid &&
            (now >= _jmit_baro_timestamp) &&
            ((now - _jmit_baro_timestamp) <= 500000ULL) &&
            PX4_ISFINITE(_jmit_baro_alt_m);

        const double trusted_n = lpos_xy_recent ? _lpos_n_m : meas_n;
        const double trusted_e = lpos_xy_recent ? _lpos_e_m : meas_e;
        const double trusted_alt = lpos_alt_recent ?
            _jmit_lpos_alt_msl :
            (baro_alt_recent ?
                static_cast<double>(_jmit_baro_alt_m) + _jmit_baro_alt_offset :
                meas_alt);

        // Inicializacao do filtro com a primeira estimativa confiavel disponivel.
        // Durante a mitigacao, a inicializacao prioriza estimativas recentes de
        // posicao local e altitude barometrica quando disponiveis.
        if (!_kf_initialized) {
            _kf_n_m   = _attack_mitigation_enabled ? trusted_n : meas_n;
            _kf_e_m   = _attack_mitigation_enabled ? trusted_e : meas_e;
            _kf_alt   = _attack_mitigation_enabled ? trusted_alt : meas_alt;
            _kf_vel_n = vel_north;
            _kf_vel_e = vel_east;
            _kf_vel_d = vel_down;
            _kf_P_n   = JMIT_R_POS_CLEAN;
            _kf_P_e   = JMIT_R_POS_CLEAN;
            _kf_P_alt = JMIT_R_ALT_CLEAN;
            _kf_P_vn  = static_cast<float>(JMIT_R_VEL_CLEAN);
            _kf_P_ve  = static_cast<float>(JMIT_R_VEL_CLEAN);
            _kf_P_vd  = static_cast<float>(JMIT_R_VEL_CLEAN);
            _kf_attack_dur_us = 0;
            _kf_last_us       = now;
            _kf_initialized   = true;

        } else {
            const double dt = math::constrain(
                static_cast<double>(now - _kf_last_us) * 1e-6, 0.005, 0.25);
            _kf_last_us = now;

            // Predicao cinematica: posicao esperada a partir da velocidade estimada.
            const double pred_n   = _kf_n_m  + static_cast<double>(_kf_vel_n) * dt;
            const double pred_e   = _kf_e_m  + static_cast<double>(_kf_vel_e) * dt;
            const double pred_alt = _kf_alt  - static_cast<double>(_kf_vel_d) * dt;

            // Crescimento limitado da incerteza para manter o teste de inovacao numericamente estavel.
            const double P_n   = math::min(_kf_P_n   + JMIT_Q_POS, static_cast<double>(JMIT_P_MAX));
            const double P_e   = math::min(_kf_P_e   + JMIT_Q_POS, static_cast<double>(JMIT_P_MAX));
            const double P_alt = math::min(_kf_P_alt + JMIT_Q_ALT, static_cast<double>(JMIT_P_MAX));
            const float  P_vn  = math::min(_kf_P_vn  + static_cast<float>(JMIT_Q_VEL), JMIT_P_MAX);
            const float  P_ve  = math::min(_kf_P_ve  + static_cast<float>(JMIT_Q_VEL), JMIT_P_MAX);
            const float  P_vd  = math::min(_kf_P_vd  + static_cast<float>(JMIT_Q_VEL), JMIT_P_MAX);

            // Inovacao: diferenca entre medicao GPS recebida e predicao do filtro.
            const double inn_n   = meas_n   - pred_n;
            const double inn_e   = meas_e   - pred_e;
            const double inn_alt = meas_alt - pred_alt;

            // NIS alto indica que o GPS esta estatisticamente incompativel com a predicao.
            const double NIS_n   = (inn_n   * inn_n)   / (P_n   + JMIT_R_POS_CLEAN);
            const double NIS_e   = (inn_e   * inn_e)   / (P_e   + JMIT_R_POS_CLEAN);
            const double NIS_alt = (inn_alt * inn_alt) / (P_alt + JMIT_R_ALT_CLEAN);

            const bool gps_anomalous =
                (NIS_n   > JMIT_NIS_THRESHOLD) ||
                (NIS_e   > JMIT_NIS_THRESHOLD) ||
                (NIS_alt > JMIT_NIS_THRESHOLD);
            gps_anomalous_now = gps_anomalous;

            if (_attack_mitigation_enabled && gps_anomalous) {
                const double max_nis = math::max(NIS_n, math::max(NIS_e, NIS_alt));

                if (max_nis >= AUTO_LAND_SEVERE_NIS_THRESHOLD) {
                    _gps_anomaly_severe = true;
                    _gps_anomaly_severe_us = now;
                }
            }

            // Se a mitigacao estiver ativa, medicoes anomalas sao rejeitadas.
            if (_attack_mitigation_enabled && gps_anomalous) {

                const bool vio_recent_for_dr =
                    _vio_valid &&
                    (now >= _vio_timestamp) &&
                    ((now - _vio_timestamp) <= BLACKOUT_VIO_TIMEOUT_US) &&
                    PX4_ISFINITE(_vio_n_m) &&
                    PX4_ISFINITE(_vio_e_m) &&
                    PX4_ISFINITE(_vio_alt_msl);

                const float dr_vel_n = vio_recent_for_dr ?
                    math::constrain(_vio_vel_n, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S) : 0.0f;

                const float dr_vel_e = vio_recent_for_dr ?
                    math::constrain(_vio_vel_e, -JMIT_DR_VEL_MAX_M_S, JMIT_DR_VEL_MAX_M_S) : 0.0f;

                const float dr_vel_d = vio_recent_for_dr ?
                    math::constrain(_vio_vel_d, -JMIT_DR_VEL_Z_MAX_M_S, JMIT_DR_VEL_Z_MAX_M_S) : 0.0f;

                if (_kf_attack_first) {
                    _jmit_pub_n_m = vio_recent_for_dr ? _vio_n_m : (lpos_xy_recent ? _lpos_n_m : _kf_n_m);
                    _jmit_pub_e_m = vio_recent_for_dr ? _vio_e_m : (lpos_xy_recent ? _lpos_e_m : _kf_e_m);
                    _jmit_pub_alt = vio_recent_for_dr ? _vio_alt_msl : trusted_alt;

                    _jmit_pub_vel_n = dr_vel_n;
                    _jmit_pub_vel_e = dr_vel_e;
                    _jmit_pub_vel_d = dr_vel_d;

                    _kf_attack_first = false;
                }

                _kf_attack_dur_us += static_cast<uint64_t>(dt * 1e6);

                if (vio_recent_for_dr) {
                    _jmit_pub_n_m = _vio_n_m;
                    _jmit_pub_e_m = _vio_e_m;
                    _jmit_pub_alt = _vio_alt_msl;

                    _jmit_pub_vel_n = dr_vel_n;
                    _jmit_pub_vel_e = dr_vel_e;
                    _jmit_pub_vel_d = dr_vel_d;

                } else {
                    _jmit_pub_vel_n = 0.0f;
                    _jmit_pub_vel_e = 0.0f;
                    _jmit_pub_vel_d = 0.0f;

                    if (!PX4_ISFINITE(_jmit_pub_alt)) {
                        _jmit_pub_alt = _kf_alt;
                    }
                }

                _kf_n_m   = _jmit_pub_n_m;
                _kf_e_m   = _jmit_pub_e_m;
                _kf_alt   = _jmit_pub_alt;
                _kf_vel_n = _jmit_pub_vel_n;
                _kf_vel_e = _jmit_pub_vel_e;
                _kf_vel_d = _jmit_pub_vel_d;

                _kf_P_n   = JMIT_R_POS_CLEAN;
                _kf_P_e   = JMIT_R_POS_CLEAN;
                _kf_P_alt = JMIT_R_ALT_CLEAN;
                _kf_P_vn  = static_cast<float>(JMIT_R_VEL_CLEAN);
                _kf_P_ve  = static_cast<float>(JMIT_R_VEL_CLEAN);
                _kf_P_vd  = static_cast<float>(JMIT_R_VEL_CLEAN);

                // Log periodico de diagnostico do modo de mitigacao.
                static uint64_t _kf_log_last_us = 0;
                if (now - _kf_log_last_us > 1000000ULL) {
                    _kf_log_last_us = now;
                    PX4_INFO("\n[KF-Mitig] ANCHOR-DR | dur=%.1f s | NIS=(%.0f,%.0f,%.0f) | pos=(%.1f,%.1f) m | alt=%.2f | alt_src=%s | vel_src=%s | vel=(%.1f,%.1f,%.1f) m/s\n",
                        static_cast<double>(_kf_attack_dur_us) / 1e6,
                        NIS_n, NIS_e, NIS_alt,
                        _jmit_pub_n_m, _jmit_pub_e_m,
                        _jmit_pub_alt,
                        vio_recent_for_dr ? "VIO" : "HOLD",
                        vio_recent_for_dr ? "VIO" : "HOLD",
                        static_cast<double>(_jmit_pub_vel_n),
                        static_cast<double>(_jmit_pub_vel_e),
                        static_cast<double>(_jmit_pub_vel_d));
                }

                // Reprojecao da ancora local para latitude/longitude antes da publicacao GPS.
                double anchor_lat, anchor_lon;
                _pos_ref.reproject(
                    static_cast<float>(_jmit_pub_n_m),
                    static_cast<float>(_jmit_pub_e_m),
                    anchor_lat, anchor_lon);
                latitude  = anchor_lat;
                longitude = anchor_lon;
                altitude  = _jmit_pub_alt;
                // Publica velocidade consistente com a propagacao da ancora.
                // Com odometria visual expirada, publica velocidade zero para manter hold.
                vel_north = _jmit_pub_vel_n;
                vel_east  = _jmit_pub_vel_e;
                vel_down  = _jmit_pub_vel_d;

            } else {

                // Modo nominal: a medicao e aceita e usada para atualizar o filtro.
                _kf_attack_dur_us = 0;
                _kf_attack_first  = true;

                if (!_gps_landing_active && !_gps_auto_land_sent) {
                    _gps_anomaly_severe = false;
                    _gps_anomaly_severe_us = 0;
                    _gps_land_source_severe_anomaly = false;
                }

                // Ganhos de Kalman para posicao, altitude e velocidade.
                const double K_n   = P_n   / (P_n   + JMIT_R_POS_CLEAN);
                const double K_e   = P_e   / (P_e   + JMIT_R_POS_CLEAN);
                const double K_alt = P_alt / (P_alt + JMIT_R_ALT_CLEAN);
                const float  K_vn  = P_vn  / (P_vn  + static_cast<float>(JMIT_R_VEL_CLEAN));
                const float  K_ve  = P_ve  / (P_ve  + static_cast<float>(JMIT_R_VEL_CLEAN));
                const float  K_vd  = P_vd  / (P_vd  + static_cast<float>(JMIT_R_VEL_CLEAN));

                const float  inn_vn  = vel_north - _kf_vel_n;
                const float  inn_ve  = vel_east  - _kf_vel_e;
                const float  inn_vd  = vel_down  - _kf_vel_d;

                _kf_n_m   = pred_n   + K_n   * inn_n;
                _kf_e_m   = pred_e   + K_e   * inn_e;
                _kf_alt   = pred_alt + K_alt * inn_alt;

                // Calibracao auxiliar entre altitude filtrada e altitude barometrica.
                if (_jmit_baro_valid) {
                    _jmit_baro_alt_offset =
                        _kf_alt - static_cast<double>(_jmit_baro_alt_m);

                    _jmit_baro_alt_offset_valid = true;
                }
                _kf_vel_n = _kf_vel_n + K_vn * inn_vn;
                _kf_vel_e = _kf_vel_e + K_ve * inn_ve;
                _kf_vel_d = _kf_vel_d + K_vd * inn_vd;

                _kf_P_n   = (1.0  - K_n  ) * P_n;
                _kf_P_e   = (1.0  - K_e  ) * P_e;
                _kf_P_alt = (1.0  - K_alt) * P_alt;
                _kf_P_vn  = (1.0f - K_vn ) * P_vn;
                _kf_P_ve  = (1.0f - K_ve ) * P_ve;
                _kf_P_vd  = (1.0f - K_vd ) * P_vd;

            }
        }
    }
    // ======== MITIGACAO GPS - FIM ========

    // ======== MITIGACAO DE POUSO GPS - INICIO ========
    checkGpsAutoLand(timestamp);

    const bool anomaly_preland_active =
        _attack_mitigation_enabled &&
        !_gps_landing_active &&
        !_gps_auto_land_sent &&
        (_gps_auto_land_arrival_us != 0) &&
        _kf_initialized &&
        (_kf_attack_dur_us >= AUTO_LAND_MIN_NOISE_US) &&
        !_blackout_anchor_initialized;

    if (anomaly_preland_active) {
        double preland_alt_msl = _gps_land_start_alt_msl;

        if (_gps_land_last_alt_valid && PX4_ISFINITE(_gps_land_last_alt_msl)) {
            preland_alt_msl = _gps_land_last_alt_msl;
        }

        _jmit_pub_n_m = _gps_land_hold_n_m;
        _jmit_pub_e_m = _gps_land_hold_e_m;
        _jmit_pub_alt = preland_alt_msl;
        _jmit_pub_vel_n = 0.0f;
        _jmit_pub_vel_e = 0.0f;
        _jmit_pub_vel_d = 0.0f;

        _kf_n_m = _jmit_pub_n_m;
        _kf_e_m = _jmit_pub_e_m;
        _kf_alt = _jmit_pub_alt;
        _kf_vel_n = 0.0f;
        _kf_vel_e = 0.0f;
        _kf_vel_d = 0.0f;

        double hold_lat = 0.0;
        double hold_lon = 0.0;
        _pos_ref.reproject(
            static_cast<float>(_gps_land_hold_n_m),
            static_cast<float>(_gps_land_hold_e_m),
            hold_lat,
            hold_lon);

        latitude = hold_lat;
        longitude = hold_lon;
        altitude = preland_alt_msl;
        vel_north = 0.0f;
        vel_east = 0.0f;
        vel_down = 0.0f;
        gps_anomalous_now = true;
    }

    if (_gps_landing_active) {
        const bool land_att_recent =
            _sim_att_valid &&
            (timestamp >= _sim_att_timestamp) &&
            ((timestamp - _sim_att_timestamp) <= 500000ULL);

        const float land_abs_pitch_rad = land_att_recent ?
            fabsf(_sim_pitch_rad) : 0.0f;

        const float land_abs_roll_rad = land_att_recent ?
            fabsf(_sim_roll_rad) : 0.0f;

        const bool descent_attitude_ok =
            !land_att_recent ||
            ((land_abs_pitch_rad <= AUTO_LAND_MAX_PITCH_RAD) &&
            (land_abs_roll_rad <= AUTO_LAND_MAX_ROLL_RAD));

        double land_alt_msl = _gps_land_last_alt_valid ?
            _gps_land_last_alt_msl : _gps_land_start_alt_msl;

        float land_vel_d = 0.0f;

        const double target_alt_msl = _gps_land_source_blackout ?
            _gps_land_target_alt_msl :
            static_cast<double>(_alt_ref) + AUTO_LAND_TARGET_GPS_ANOMALY_AGL_M;

        const double dt_land_s =
            (_gps_land_last_update_us > 0 && timestamp >= _gps_land_last_update_us) ?
            math::constrain(static_cast<double>(timestamp - _gps_land_last_update_us) * 1e-6, 0.0, 0.1) :
            0.0;

        _gps_land_last_update_us = timestamp;

        if (descent_attitude_ok && !_gps_auto_disarm_sent) {
            land_alt_msl = math::max(
                target_alt_msl,
                land_alt_msl - (AUTO_LAND_DESCENT_RATE_M_S * dt_land_s));

            land_vel_d = (land_alt_msl <= target_alt_msl) ?
                0.0f : static_cast<float>(AUTO_LAND_DESCENT_RATE_M_S);
        }

        if (_gps_auto_disarm_sent) {
            land_alt_msl = target_alt_msl;
            land_vel_d = 0.0f;
        }

        _gps_land_last_alt_msl = land_alt_msl;
        _gps_land_last_alt_valid = true;

        static uint64_t land_att_hold_log_last_us = 0;

        if (!descent_attitude_ok &&
                ((timestamp - land_att_hold_log_last_us) > AUTO_LAND_ATT_LOG_INTERVAL_US)) {
            land_att_hold_log_last_us = timestamp;

            PX4_INFO("\n[GPS-Land] Holding descent due attitude | pitch=%.1f deg | roll=%.1f deg | alt=%.2f\n",
                land_att_recent ? static_cast<double>(_sim_pitch_rad * 57.2957795f) : -999.0,
                land_att_recent ? static_cast<double>(_sim_roll_rad * 57.2957795f) : -999.0,
                land_alt_msl);
        }

        _jmit_pub_n_m = _gps_land_hold_n_m;
        _jmit_pub_e_m = _gps_land_hold_e_m;
        _jmit_pub_alt = land_alt_msl;
        _jmit_pub_vel_n = 0.0f;
        _jmit_pub_vel_e = 0.0f;
        _jmit_pub_vel_d = land_vel_d;

        _kf_n_m = _jmit_pub_n_m;
        _kf_e_m = _jmit_pub_e_m;
        _kf_alt = _jmit_pub_alt;
        _kf_vel_n = 0.0f;
        _kf_vel_e = 0.0f;
        _kf_vel_d = land_vel_d;

        _blackout_dr_n_m = _gps_land_hold_n_m;
        _blackout_dr_e_m = _gps_land_hold_e_m;
        _blackout_dr_alt = land_alt_msl;
        _blackout_dr_vel_n = 0.0f;
        _blackout_dr_vel_e = 0.0f;
        _blackout_dr_vel_d = land_vel_d;

        double land_lat = 0.0;
        double land_lon = 0.0;
        _pos_ref.reproject(
            static_cast<float>(_gps_land_hold_n_m),
            static_cast<float>(_gps_land_hold_e_m),
            land_lat,
            land_lon);

        latitude = land_lat;
        longitude = land_lon;
        altitude = land_alt_msl;
        vel_north = 0.0f;
        vel_east = 0.0f;
        vel_down = land_vel_d;
    }

    checkGpsAutoDisarm(timestamp);
    // ======== MITIGACAO DE POUSO GPS - FIM ========

    // ======== ATAQUE 3 GPS PULSED JAMMING - INICIO ========
    // Ataque 3: maquina de estados do jamming pulsado.
    // if (_jamming_attack_type == 3 && _jamming_pulsed_initialized) {
        // const uint64_t now     = hrt_absolute_time();
        // const uint64_t elapsed = now - _pjam_state_enter_us;

        // Estados: subida do jammer, bloqueio total, decaimento e periodo valido.
        // switch (_pjam_state) {

        // case PulsedJamState::RAMP_UP: {
            // RAMP_UP: o sinal piora gradualmente e o ruido cresce de 0 ate 100%.

            // _pjam_noise_factor = math::constrain(
                // static_cast<float>(elapsed) / static_cast<float>(_pjam_ramp_us),
                // 0.0f, 1.0f);
            // _jamming_intermittent_active = true;

            // if (elapsed >= _pjam_ramp_us) {
                // _pjam_state          = PulsedJamState::JAM;
                // _pjam_state_enter_us = now;
                // _pjam_noise_factor   = 1.0f;
                // PX4_WARN("[UAVJamSim Jamming] >>> JAM ON <<< (%.1f s)",
                         // static_cast<double>(_pjam_on_us) / 1e6);
            // }

            // break;
        // }

        // case PulsedJamState::JAM: {
            // JAM: bloqueio total; nenhuma mensagem GPS e publicada enquanto o pulso esta ativo.
            // Durante o pouso ja iniciado, mantem a pseudo-medicao de pouso sendo publicada.
            // _pjam_noise_factor           = 1.0f;
            // _jamming_intermittent_active = true;

            // if (elapsed >= _pjam_on_us) {
                // _pjam_state          = PulsedJamState::DECAY;
                // _pjam_state_enter_us = now;
                // PX4_INFO("[UAVJamSim Jamming] <<< JAM OFF — DECAY >>>");
            // } else {
                // if (!_gps_landing_active) {
                    // return;
                // }
            // }
            // break;
        // }

        // case PulsedJamState::DECAY: {
            // DECAY: o jammer deixa de bloquear e o ruido reduz gradualmente.

            // _pjam_noise_factor = math::constrain(
                // 1.0f - static_cast<float>(elapsed) / static_cast<float>(_pjam_decay_us),
                // 0.0f, 1.0f);
            // _jamming_intermittent_active = (_pjam_noise_factor > 0.05f);

            // if (elapsed >= _pjam_decay_us) {
                // _pjam_state          = PulsedJamState::VALID;
                // _pjam_state_enter_us = now;
                // _pjam_noise_factor   = 0.0f;
                // _jamming_intermittent_active = false;
                // PX4_INFO("[UAVJamSim Jamming] <<< GPS VALID — recovery >>>");
            // }
            // break;
        // }

        // case PulsedJamState::VALID: {
            // VALID: janela com GPS limpo antes do proximo pulso.
            // _pjam_noise_factor           = 0.0f;
            // _jamming_intermittent_active = false;

            // if (elapsed >= _pjam_off_us) {

                // _pjam_state          = PulsedJamState::RAMP_UP;
                // _pjam_state_enter_us = now;
                // PX4_WARN("[UAVJamSim Jamming] >>> NEW PULSE — RAMP_UP >>>");
            // }
            // break;
        // }

        // default:
            // break;
        // }

        // Em RAMP_UP/DECAY, injeta ruido proporcional a potencia instantanea do pulso.
        // Durante o pouso, mantem a pseudo-medicao sem ruido pulsado adicional.
        // if (_pjam_noise_factor > 0.0f) {
            // const float noise_factor = _pjam_noise_factor;
            // const double lat_rad = math::radians(latitude);
            // const double noise_n_m = static_cast<double>(generate_wgn() * noise_factor * _jamming_intensity * 1000.0f);
            // const double noise_e_m = static_cast<double>(generate_wgn() * noise_factor * _jamming_intensity * 1000.0f);

            // latitude  += math::degrees(noise_n_m / CONSTANTS_RADIUS_OF_EARTH);
            // longitude += math::degrees(noise_e_m / (CONSTANTS_RADIUS_OF_EARTH * cos(lat_rad)));
            // altitude  += static_cast<double>(generate_wgn() * noise_factor * _jamming_intensity * 80.0f);
            // vel_north += generate_wgn() * noise_factor * _jamming_intensity * 15.0f;
            // vel_east  += generate_wgn() * noise_factor * _jamming_intensity * 15.0f;
            // vel_down  += generate_wgn() * noise_factor * _jamming_intensity * 15.0f;
        // }
    // }
    _jamming_attack.tickPulsedJamming(hrt_absolute_time(), latitude, longitude, altitude,
                       vel_north, vel_east, vel_down);

    // Reproduz o "return" original do estado JAM (bloqueio total do sinal
    // enquanto o pulso esta ativo), exceto durante pouso automatico ja em
    // andamento - mesma condicao que existia dentro do switch original.
    if (_jamming_attack.isPulsedFullyJammed() && !_gps_landing_active) {
        return;
    }
    // ======== ATAQUE 3 GPS PULSED JAMMING - FIM ========

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

    // ======== ATAQUE 3 GPS PULSED JAMMING - INICIO ========
    // No jamming pulsado, degrada eph/epv/HDOP/VDOP e reduz satelites durante RAMP_UP/DECAY.
    // if (_pjam_noise_factor > 0.0f) {
        // const float noise_factor = _pjam_noise_factor;

        // sensor_gps.eph  = sensor_gps.eph  + noise_factor * (15.0f  - sensor_gps.eph);
        // sensor_gps.epv  = sensor_gps.epv  + noise_factor * (25.0f  - sensor_gps.epv);
        // sensor_gps.hdop = sensor_gps.hdop + noise_factor * (8.0f   - sensor_gps.hdop);
        // sensor_gps.vdop = sensor_gps.vdop + noise_factor * (12.0f  - sensor_gps.vdop);

        // int base_satellites = _sim_gps_used.get();
        // sensor_gps.satellites_used = static_cast<uint8_t>(
            // math::constrain(static_cast<int>(base_satellites - static_cast<int>(noise_factor * (base_satellites - 3))),
                            // 3, base_satellites));
    // } else {
        // sensor_gps.satellites_used = _sim_gps_used.get();
    // }
    // ======== ATAQUE 3 GPS PULSED JAMMING - FIM ========

    // ======== QUALIDADE GPS DURANTE NOISE JAMMING - INICIO ========
    // O GPS ruidoso continua valido, mas com incerteza maior para evitar
    // que o estimador trate saltos grandes como medicoes precisas.
    // if (_jamming_attack_type == 1) {
        // sensor_gps.eph  = math::max(sensor_gps.eph, 8.0f);
        // sensor_gps.epv  = math::max(sensor_gps.epv, 12.0f);
        // sensor_gps.hdop = math::max(sensor_gps.hdop, 2.5f);
        // sensor_gps.vdop = math::max(sensor_gps.vdop, 3.5f);
    // }
    // ======== QUALIDADE GPS DURANTE NOISE JAMMING - FIM ========

    // Encanamento: os dois blocos comentados acima ja estao implementados,
    // idênticos, dentro de JammingAttack::degradeGpsQuality (inclui o
    // fallback nominal de satellites_used quando nenhum ataque esta ativo).
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
    if (_gps_landing_active && _gps_land_heading_valid &&
            fabsf(vel_north) < 0.05f && fabsf(vel_east) < 0.05f) {
        sensor_gps.cog_rad = _gps_land_heading_rad;
    } else {
        sensor_gps.cog_rad = atan2(vel_east, vel_north);
    }

    // ======== MITIGACAO GPS - INICIO ========
    // Em modo ancora, a pseudo-medicao GPS e marcada como degradada, mas continua publicada.
    const bool is_anchor_now = _attack_mitigation_enabled && ((_kf_attack_dur_us > 0) || _gps_landing_active || _gps_auto_disarm_sent);
    const bool landing_now = _gps_landing_active || _gps_auto_disarm_sent;

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
    // if (!is_anchor_now && !_jamming_intermittent_active) {
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
        // em imuCallback como referencia cruzada para o acelerometro.
        if (_imu_ref_gps_prev_valid && (timestamp > _imu_ref_gps_prev_us)) {
            const double dt_ref = static_cast<double>(timestamp - _imu_ref_gps_prev_us) * 1e-6;

            if (dt_ref >= IMU_REF_GPS_MIN_DT_S && dt_ref <= IMU_REF_GPS_MAX_DT_S) {
                const double accel_n_est =
                    (static_cast<double>(vel_north) - static_cast<double>(_imu_ref_gps_prev_vel_n)) / dt_ref;
                const double accel_e_est =
                    (static_cast<double>(vel_east) - static_cast<double>(_imu_ref_gps_prev_vel_e)) / dt_ref;

                if (PX4_ISFINITE(accel_n_est) && PX4_ISFINITE(accel_e_est)) {
                    _imu_ref_gps_accel_n = static_cast<float>(accel_n_est);
                    _imu_ref_gps_accel_e = static_cast<float>(accel_e_est);
                    _imu_ref_gps_accel_valid = true;
                    _imu_ref_gps_accel_us = timestamp;
                }
            }
        }

        _imu_ref_gps_prev_vel_n = vel_north;
        _imu_ref_gps_prev_vel_e = vel_east;
        _imu_ref_gps_prev_us = timestamp;
        _imu_ref_gps_prev_valid = true;
        // ======== MITIGACAO IMU - FIM ========
    }
    // ======== MITIGACAO BLACKOUT GPS - FIM ========

    _sensor_gps_pub.publish(sensor_gps);
}

// Extrai a profundidade media de uma pequena janela central da imagem
// (mesma direcao do LiDAR frontal), usada como referencia cruzada
// independente pela mitigacao. So o x500_uavjamsim publica esse topico.
// Amostra a profundidade media de um patch ~7x7 pixels ao redor de (cx, cy).
// Retorna NAN se a janela cair fora da imagem ou nao encontrar pixel valido.
float GZBridge::samplePatchDepth(const gz::msgs::Image &msg, int cx, int cy, int patch_half)
{
    const int width = static_cast<int>(msg.width());
    const int height = static_cast<int>(msg.height());

    if (width <= 0 || height <= 0) {
        return NAN;
    }

    const char *data = msg.data().data();
    const size_t data_size = msg.data().size();

    double sum = 0.0;
    int count = 0;

    for (int dy = -patch_half; dy <= patch_half; dy++) {
        const int y = cy + dy;

        if (y < 0 || y >= height) {
            continue;
        }

        for (int dx = -patch_half; dx <= patch_half; dx++) {
            const int x = cx + dx;

            if (x < 0 || x >= width) {
                continue;
            }

            const size_t offset = (static_cast<size_t>(y) * width + x) * sizeof(float);

            if (offset + sizeof(float) > data_size) {
                continue;
            }

            float depth = 0.0f;
            memcpy(&depth, data + offset, sizeof(float));

            if (PX4_ISFINITE(depth) && depth > 0.0f) {
                sum += static_cast<double>(depth);
                count++;
            }
        }
    }

    return (count > 0) ? static_cast<float>(sum / count) : NAN;
}

void GZBridge::depthCameraCallback(const gz::msgs::Image &msg)
{
    const uint64_t timestamp = hrt_absolute_time();

    const int width = static_cast<int>(msg.width());
    const int height = static_cast<int>(msg.height());

    if (width <= 0 || height <= 0) {
        return;
    }

    const int patch_half = 3; // janela ~7x7 pixels
    const int cx = width / 2;
    const int cy = height / 2;

    // Leitura central (usada pela mitigacao do LiDAR frontal).
    const float center_depth = samplePatchDepth(msg, cx, cy, patch_half);

    if (PX4_ISFINITE(center_depth)) {
        _depth_cam_distance_m = center_depth;
        _depth_cam_distance_valid = true;
        _depth_cam_distance_timestamp = timestamp;
    }

    // Leitura por setor (usada pela mitigacao do LiDAR 2D): projeta cada
    // angulo de setor coberto na coluna de pixel correspondente, usando o
    // campo de visao horizontal conhecido da camera (StereoOV7251, ver
    // OakD-Lite/model.sdf), e amostra um patch nessa coluna.
    static constexpr double CAMERA_HFOV_RAD = 1.274;
    static constexpr double LIDAR2D_SECTOR_DEG = 5.0;

    const double focal_px = (static_cast<double>(width) / 2.0) / tan(CAMERA_HFOV_RAD / 2.0);
    bool any_sector_valid = false;

    for (int k = -LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS; k <= LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS; k++) {
        const double angle_rad = static_cast<double>(k) * LIDAR2D_SECTOR_DEG * M_PI / 180.0;
        const int px = cx + static_cast<int>(lround(focal_px * tan(angle_rad)));
        const int idx = k + LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS;

        const float sector_depth = samplePatchDepth(msg, px, cy, patch_half);
        _depth_cam_sector_distance_m[idx] = sector_depth;

        if (PX4_ISFINITE(sector_depth)) {
            any_sector_valid = true;
        }
    }

    if (any_sector_valid) {
        _depth_cam_sector_valid = true;
        _depth_cam_sector_timestamp = timestamp;
    }
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

    // ======== ATAQUE LIDAR - INICIO ========
    // Ataque LiDAR: soma offset a medicao de distancia.
    if (_lidar_attack_option == 1) {
        processed_dist += static_cast<float>(_lidar_distance_offset);
    }
    // ======== ATAQUE LIDAR - FIM ========

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
    // Deteccao de anomalia no sinal publicado do sensor, sem consultar
    // _lidar_attack_option:
    //  (a) leitura fora da faixa fisica funcional do sensor;
    //  (b) leitura divergindo do valor esperado por uma ancora entre a
    //      leitura do LiDAR e a altitude do barometro (referencia
    //      independente - este airframe nao tem VIO confiavel), calibrada a
    //      partir das primeiras amostras saudaveis. O ataque e um offset
    //      constante, que uma comparacao por taxa de variacao nunca pegaria
    //      (o offset se cancela na diferenca entre amostras): so uma
    //      comparacao por valor absoluto contra essa ancora detecta.
    // So se aplica ao sensor apontado para baixo (terrain-following); um
    // LiDAR front-facing, se existir, mede distancia de obstaculo, nao altura.
    // Diferente das demais mitigacoes, nao exige que o veiculo ja tenha
    // armado/voado: a leitura do LiDAR e significativa mesmo parado no chao
    // (mede a folga ate o solo), e o ataque deve ser detectavel nesse estado.
    if (touchdown_range_sensor) {

        if (_attack_mitigation_enabled) {

            // Nao ha checagem util de limite fisico aqui: report.current_distance
            // ja passou por math::constrain(min, max) alguns comandos acima, entao
            // uma leitura saturada em min/max e absolutamente normal (nada no
            // alcance, ou muito perto) e indistinguivel de uma leitura atacada
            // empurrada para fora da faixa - as duas terminam no mesmo valor
            // limitado. So a comparacao com a ancora abaixo consegue distinguir.
            const bool non_finite = !PX4_ISFINITE(report.current_distance);

            const bool baro_alt_recent =
                _jmit_baro_valid &&
                (timestamp >= _jmit_baro_timestamp) &&
                ((timestamp - _jmit_baro_timestamp) <= LIDAR_BARO_MAX_AGE_US) &&
                PX4_ISFINITE(_jmit_baro_alt_m);

            double residual = 0.0;
            bool residual_anomaly = false;

            if (baro_alt_recent && _lidar_baseline_initialized) {
                const double expected_dist = _lidar_anchor_offset_m + static_cast<double>(_jmit_baro_alt_m);
                residual = static_cast<double>(report.current_distance) - expected_dist;
                residual_anomaly = fabs(residual) > LIDAR_RESIDUAL_THRESHOLD_M;
            }

            const bool sample_anomalous = non_finite || residual_anomaly;
            const bool sample_severe = non_finite;

            if (!sample_anomalous) {

                // Atualiza a ancora apenas com amostras saudaveis. As
                // primeiras amostras (warmup) calibram com um alpha mais
                // agressivo; depois, adapta lentamente (tolera deriva
                // legitima de calibracao/temperatura, mas um ataque
                // sustentado nao chega a ser absorvido, pois so entra aqui
                // quando a amostra nao e anomala).
                if (baro_alt_recent) {
                    const double sample_offset = static_cast<double>(report.current_distance) - static_cast<double>(_jmit_baro_alt_m);

                    if (_lidar_baseline_warmup_count == 0) {
                        _lidar_anchor_offset_m = sample_offset;
                        _lidar_baseline_warmup_count = 1;

                    } else if (_lidar_baseline_warmup_count < LIDAR_BASELINE_WARMUP_SAMPLES) {
                        _lidar_anchor_offset_m += LIDAR_BASELINE_WARMUP_ALPHA * (sample_offset - _lidar_anchor_offset_m);
                        _lidar_baseline_warmup_count++;

                        if (_lidar_baseline_warmup_count >= LIDAR_BASELINE_WARMUP_SAMPLES) {
                            _lidar_baseline_initialized = true;
                        }

                    } else {
                        _lidar_anchor_offset_m += LIDAR_BASELINE_ALPHA * (sample_offset - _lidar_anchor_offset_m);
                    }
                }

                if (_lidar_anomaly_active) {
                    if (_lidar_recovery_start_us == 0) {
                        _lidar_recovery_start_us = timestamp;

                    } else if ((timestamp - _lidar_recovery_start_us) >= LIDAR_RECOVERY_CONFIRM_US) {
                        _lidar_anomaly_active = false;
                        _lidar_anomaly_start_us = 0;
                        _lidar_mitig_diag_count = 0;

                        PX4_INFO("\n[Lidar-Mitig] Sensor anomaly cleared\n");
                    }
                }

            } else {
                _lidar_recovery_start_us = 0;

                if (_lidar_anomaly_start_us == 0) {
                    _lidar_anomaly_start_us = timestamp;
                }

                const uint64_t confirm_required_us =
                    sample_severe ? LIDAR_ANOMALY_CONFIRM_SEVERE_US : LIDAR_ANOMALY_CONFIRM_US;

                if (!_lidar_anomaly_active &&
                        ((timestamp - _lidar_anomaly_start_us) >= confirm_required_us)) {
                    _lidar_anomaly_active = true;

                    PX4_WARN("\n[Lidar-Mitig] Sensor anomaly detected | dist=%.2f m | residual=%.2f m | finite=%s\n",
                        static_cast<double>(report.current_distance),
                        residual,
                        non_finite ? "no" : "yes");
                }

                // Reconstroi a leitura a partir da ancora e da altitude atual
                // do barometro, preservando a fisica do sensor (nunca fora
                // dos limites). So corrige quando o barometro e a ancora ja
                // calibrada estao disponiveis; sem isso, deixa a leitura
                // crua passar (melhor que travar em um valor desatualizado).
                if (_lidar_anomaly_active && baro_alt_recent && _lidar_baseline_initialized) {
                    const double expected_dist = _lidar_anchor_offset_m + static_cast<double>(_jmit_baro_alt_m);
                    report.current_distance = static_cast<float>(math::constrain(
                        expected_dist,
                        static_cast<double>(report.min_distance),
                        static_cast<double>(report.max_distance)));
                }

                if (_lidar_anomaly_active && (_lidar_mitig_diag_count < LIDAR_MITIG_DIAG_MAX_COUNT)) {
                    static uint64_t lidar_mitig_diag_last_us = 0;

                    if ((timestamp - lidar_mitig_diag_last_us) > 1000000ULL) {
                        lidar_mitig_diag_last_us = timestamp;
                        _lidar_mitig_diag_count++;

                        PX4_INFO("\n[Lidar-Mitig] Status | dist=%.2f m | residual=%.2f m | baro_alt=%s\n",
                            static_cast<double>(report.current_distance),
                            residual,
                            baro_alt_recent ? "ok" : "stale");

                        if (_lidar_mitig_diag_count == LIDAR_MITIG_DIAG_MAX_COUNT) {
                            PX4_WARN("\n[Lidar-Mitig] Check: listener distance_sensor (down, inst 0)\n");
                        }
                    }
                }
            }
        }
    }
    // ======== MITIGACAO LIDAR - FIM ========

    // ======== MITIGACAO LIDAR FRONTAL - INICIO ========
    // Deteccao de anomalia comparando a leitura publicada contra a camera de
    // profundidade (OakD-Lite, so presente no x500_uavjamsim), referencia
    // independente do LiDAR. Diferente da mitigacao do LiDAR para baixo, os
    // dois sensores medem a mesma coisa em tempo real (nao e um offset
    // constante que se cancela na diferenca entre amostras aqui, ja que a
    // camera nao esta sob o mesmo ataque): e uma comparacao direta, sem
    // ancora/calibracao. Sem essa camera disponivel (qualquer outro
    // airframe), a deteccao simplesmente nao roda — nao ha limite fisico
    // util aqui pelo mesmo motivo do LiDAR para baixo (saturacao no maximo
    // e indistinguivel de ataque empurrando para fora da faixa).
    if (report.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) {

        if (_attack_mitigation_enabled) {

            const bool vio_att_recent =
                _vio_heading_valid &&
                (timestamp >= _vio_heading_us) &&
                ((timestamp - _vio_heading_us) <= MAG_VIO_MAX_AGE_US);

            const bool vio_near_level =
                vio_att_recent &&
                (fabsf(_vio_roll_rad) < LIDAR_FRONT_MAX_TILT_RAD) &&
                (fabsf(_vio_pitch_rad) < LIDAR_FRONT_MAX_TILT_RAD);

            const bool depth_cam_recent =
                vio_near_level &&
                _depth_cam_distance_valid &&
                (timestamp >= _depth_cam_distance_timestamp) &&
                ((timestamp - _depth_cam_distance_timestamp) <= LIDAR_FRONT_DEPTH_MAX_AGE_US) &&
                PX4_ISFINITE(_depth_cam_distance_m);

            double residual = 0.0;
            bool sample_anomalous = false;

            if (depth_cam_recent) {
                residual = static_cast<double>(report.current_distance) - static_cast<double>(_depth_cam_distance_m);
                sample_anomalous = fabs(residual) > LIDAR_FRONT_RESIDUAL_THRESHOLD_M;
            }

            if (!sample_anomalous) {

                if (_lidar_front_anomaly_active) {
                    if (_lidar_front_recovery_start_us == 0) {
                        _lidar_front_recovery_start_us = timestamp;

                    } else if ((timestamp - _lidar_front_recovery_start_us) >= LIDAR_FRONT_RECOVERY_CONFIRM_US) {
                        _lidar_front_anomaly_active = false;
                        _lidar_front_anomaly_start_us = 0;
                        _lidar_front_mitig_diag_count = 0;

                        PX4_INFO("\n[LidarFront-Mitig] Sensor anomaly cleared\n");
                    }
                }

            } else {
                _lidar_front_recovery_start_us = 0;

                if (_lidar_front_anomaly_start_us == 0) {
                    _lidar_front_anomaly_start_us = timestamp;
                }

                if (!_lidar_front_anomaly_active &&
                        ((timestamp - _lidar_front_anomaly_start_us) >= LIDAR_FRONT_ANOMALY_CONFIRM_US)) {
                    _lidar_front_anomaly_active = true;

                    PX4_WARN("\n[LidarFront-Mitig] Sensor anomaly detected | dist=%.2f m | depth_cam=%.2f m | residual=%.2f m\n",
                        static_cast<double>(report.current_distance),
                        static_cast<double>(_depth_cam_distance_m),
                        residual);
                }

                // Substitui pela leitura da camera de profundidade,
                // preservando os limites fisicos do sensor. So corrige
                // quando a camera esta disponivel e recente; sem isso,
                // deixa a leitura crua passar.
                if (_lidar_front_anomaly_active && depth_cam_recent) {
                    report.current_distance = static_cast<float>(math::constrain(
                        static_cast<double>(_depth_cam_distance_m),
                        static_cast<double>(report.min_distance),
                        static_cast<double>(report.max_distance)));
                }

                if (_lidar_front_anomaly_active && (_lidar_front_mitig_diag_count < LIDAR_FRONT_MITIG_DIAG_MAX_COUNT)) {
                    static uint64_t lidar_front_mitig_diag_last_us = 0;

                    if ((timestamp - lidar_front_mitig_diag_last_us) > 1000000ULL) {
                        lidar_front_mitig_diag_last_us = timestamp;
                        _lidar_front_mitig_diag_count++;

                        PX4_INFO("\n[LidarFront-Mitig] Status | dist=%.2f m | residual=%.2f m | depth_cam=%s\n",
                            static_cast<double>(report.current_distance),
                            residual,
                            depth_cam_recent ? "ok" : "stale");

                        if (_lidar_front_mitig_diag_count == LIDAR_FRONT_MITIG_DIAG_MAX_COUNT) {
                            PX4_WARN("\n[LidarFront-Mitig] Check: listener distance_sensor (front, inst 1)\n");
                        }
                    }
                }
            }
        }
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

            // ======== ATAQUE LIDAR 2D - INICIO ========
            // Reaproveita o mesmo comando/estado do ataque do LiDAR 1D
            // (--lidar 1 <offset>): soma o offset a cada feixe valido antes
            // da reducao em setores.
            if (_lidar_attack_option == 1) {
                distance += _lidar_distance_offset;
            }
            // ======== ATAQUE LIDAR - FIM ========

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
    // Compara os setores dentro do alcance da camera de profundidade
    // (OakD-Lite, so presente no x500_uavjamsim) contra a leitura
    // publicada, usando o residuo MEDIO entre os setores comparaveis (nao
    // um unico setor) — o ataque aplica o mesmo offset constante a todo o
    // scan de uma vez, entao concordancia entre varios setores da mais
    // confianca que um so. Fora da faixa coberta pela camera nao ha
    // nenhuma referencia independente disponivel nesse modelo: esses
    // setores permanecem sem protecao — limitacao fisica real, nao lacuna
    // de implementacao (mesma razao ja documentada na mitigacao do LiDAR
    // frontal).
    if (_attack_mitigation_enabled) {

        const bool vio_att_recent =
            _vio_heading_valid &&
            (report.timestamp >= _vio_heading_us) &&
            ((report.timestamp - _vio_heading_us) <= MAG_VIO_MAX_AGE_US);

        const bool vio_near_level =
            vio_att_recent &&
            (fabsf(_vio_roll_rad) < LIDAR2D_MAX_TILT_RAD) &&
            (fabsf(_vio_pitch_rad) < LIDAR2D_MAX_TILT_RAD);

        const bool depth_sectors_recent =
            vio_near_level &&
            _depth_cam_sector_valid &&
            (report.timestamp >= _depth_cam_sector_timestamp) &&
            ((report.timestamp - _depth_cam_sector_timestamp) <= LIDAR2D_DEPTH_MAX_AGE_US);

        const int front_index = static_cast<int>(lroundf(-report.angle_offset / report.increment));
        const int distances_len = static_cast<int>(sizeof(report.distances) / sizeof(report.distances[0]));

        double residual_sum = 0.0;
        int residual_count = 0;

        if (depth_sectors_recent) {
            for (int k = -LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS; k <= LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS; k++) {
                const int idx = front_index + k;

                if (idx < 0 || idx >= distances_len) {
                    continue;
                }

                const float cam_depth = _depth_cam_sector_distance_m[k + LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS];

                if (!PX4_ISFINITE(cam_depth)) {
                    continue;
                }

                // So compara setores com leitura real do lidar (pula o
                // sentinela de "nada detectado"; "muito perto" (0) entra na
                // comparacao normalmente, ja que carrega informacao real).
                if (report.distances[idx] > report.max_distance) {
                    continue;
                }

                const double lidar_m = static_cast<double>(report.distances[idx]) / 100.0;
                const double residual = lidar_m - static_cast<double>(cam_depth);

                residual_sum += fabs(residual);
                residual_count++;
            }
        }

        const bool enough_data = residual_count >= LIDAR2D_MIN_VALID_SECTORS;
        const bool sample_anomalous = enough_data &&
            ((residual_sum / residual_count) > LIDAR2D_RESIDUAL_THRESHOLD_M);

        if (!sample_anomalous) {

            if (_lidar2d_anomaly_active) {
                if (_lidar2d_recovery_start_us == 0) {
                    _lidar2d_recovery_start_us = report.timestamp;

                } else if ((report.timestamp - _lidar2d_recovery_start_us) >= LIDAR2D_RECOVERY_CONFIRM_US) {
                    _lidar2d_anomaly_active = false;
                    _lidar2d_anomaly_start_us = 0;
                    _lidar2d_mitig_diag_count = 0;

                    PX4_INFO("\n[Lidar2D-Mitig] Sensor anomaly cleared\n");
                }
            }

        } else {
            _lidar2d_recovery_start_us = 0;

            if (_lidar2d_anomaly_start_us == 0) {
                _lidar2d_anomaly_start_us = report.timestamp;
            }

            if (!_lidar2d_anomaly_active &&
                    ((report.timestamp - _lidar2d_anomaly_start_us) >= LIDAR2D_ANOMALY_CONFIRM_US)) {
                _lidar2d_anomaly_active = true;

                PX4_WARN("\n[Lidar2D-Mitig] Sensor anomaly detected | residual_avg=%.2f m | setores=%d\n",
                    residual_sum / residual_count, residual_count);
            }

            // Substitui cada setor coberto pela leitura da camera,
            // respeitando os limites do sensor. So corrige os setores onde
            // a camera tem leitura valida; os demais (mesmo dentro da
            // faixa coberta) ficam com a leitura crua.
            if (_lidar2d_anomaly_active && depth_sectors_recent) {
                for (int k = -LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS; k <= LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS; k++) {
                    const int idx = front_index + k;

                    if (idx < 0 || idx >= distances_len) {
                        continue;
                    }

                    const float cam_depth = _depth_cam_sector_distance_m[k + LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS];

                    if (!PX4_ISFINITE(cam_depth)) {
                        continue;
                    }

                    const double cam_cm = static_cast<double>(cam_depth) * 100.0;

                    if (cam_cm >= static_cast<double>(report.max_distance)) {
                        report.distances[idx] = report.max_distance + 1;

                    } else if (cam_cm < static_cast<double>(report.min_distance)) {
                        report.distances[idx] = 0;

                    } else {
                        report.distances[idx] = static_cast<uint16_t>(cam_cm);
                    }
                }
            }

            if (_lidar2d_anomaly_active && (_lidar2d_mitig_diag_count < LIDAR2D_MITIG_DIAG_MAX_COUNT)) {
                static uint64_t lidar2d_mitig_diag_last_us = 0;

                if ((report.timestamp - lidar2d_mitig_diag_last_us) > 1000000ULL) {
                    lidar2d_mitig_diag_last_us = report.timestamp;
                    _lidar2d_mitig_diag_count++;

                    PX4_INFO("\n[Lidar2D-Mitig] Status | residual_avg=%.2f m | setores=%d | depth_cam=%s\n",
                        enough_data ? (residual_sum / residual_count) : -1.0,
                        residual_count,
                        depth_sectors_recent ? "ok" : "stale");

                    if (_lidar2d_mitig_diag_count == LIDAR2D_MITIG_DIAG_MAX_COUNT) {
                        PX4_WARN("\n[Lidar2D-Mitig] Check: listener obstacle_distance (front sectors)\n");
                    }
                }
            }
        }
    }
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