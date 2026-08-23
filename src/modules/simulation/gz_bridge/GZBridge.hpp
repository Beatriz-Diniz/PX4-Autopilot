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

#pragma once

#include "GZMixingInterfaceESC.hpp"
#include "GZMixingInterfaceServo.hpp"
#include "GZMixingInterfaceWheel.hpp"
#include "GZGimbal.hpp"
#include "JammingAttack.hpp"
#include "AbsoluteOffsetAttack.hpp"
#include "RotationOffsetAttack.hpp"
#include "ImuAttack.hpp"
#include "LidarAttack.hpp"
#include "BarometerAttack.hpp"
#include "MagnetometerAttack.hpp"
#include "GpsMitigation.hpp"
#include "ImuMitigation.hpp"
#include "MagnetometerMitigation.hpp"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/actuator_armed.h>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/air_speed.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/image.pb.h>
// Custom PX4 proto
#include <opticalflow.pb.h>
#include <gz/msgs/int32.pb.h>

using namespace time_literals;

class GZBridge : public ModuleBase<GZBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

    GZBridge(const std::string &world, const std::string &model_name);
    ~GZBridge() override;

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    int init();

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:

    void Run() override;

    bool subscribeClock(bool required);
    bool subscribePoseInfo(bool required);
    bool subscribeImu(bool required);
    bool subscribeMag(bool required);
    bool subscribeOdometry(bool required);
    bool subscribeLaserScan(bool required);
    bool subscribeDistanceSensor(bool required);
    // Inscricao adicional para o LiDAR frontal em modelos combinados (ex.:
    // x500_uavjamsim), onde down e front usam links/sensores com nomes
    // distintos. Reaproveita o mesmo callback de laserScantoLidarSensorCallback.
    bool subscribeDistanceSensorFront(bool required);
    bool subscribeDepthCamera(bool required);
    bool subscribeAirspeed(bool required);
    bool subscribeAirPressure(bool required);
    bool subscribeNavsat(bool required);
    bool subscribeOpticalFlow(bool required);

    // Ataques UAVJamSim
    bool subscribeAttacks(bool required);
    void motorAttackCallback(const gz::msgs::Vector3d &msg);
    void streamAttackCallback(const gz::msgs::Int32 &msg);
    // So imprime o aviso encaminhado pelo GstCameraSystem (processo do
    // Gazebo) quando a mitigacao de flip do stream confirma um ataque.
    void streamFlipDetectedCallback(const gz::msgs::Int32 &msg);
    void streamBlackDetectedCallback(const gz::msgs::Int32 &msg);

    // Mitigacao GPS
    void attackMitigationCallback(const gz::msgs::Vector3d &msg);

    void clockCallback(const gz::msgs::Clock &msg);
    void airspeedCallback(const gz::msgs::AirSpeed &msg);
    void airPressureCallback(const gz::msgs::FluidPressure &msg);

    // Monta o contexto compartilhado (VIO/LPOS/atitude/AGL/distancia/baro)
    // usado por GpsMitigation, a partir dos caches ja mantidos pelo GZBridge.
    GpsMitigationContext buildGpsMitigationContext() const;

    void imuCallback(const gz::msgs::IMU &msg);
    void poseInfoCallback(const gz::msgs::Pose_V &msg);
    void odometryCallback(const gz::msgs::OdometryWithCovariance &msg);
    void navSatCallback(const gz::msgs::NavSat &msg);
    void laserScantoLidarSensorCallbackImpl(const gz::msgs::LaserScan &msg, bool is_front_slot);
    void laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg);
    void laserScantoLidarSensorFrontCallback(const gz::msgs::LaserScan &msg);
    void depthCameraCallback(const gz::msgs::Image &msg);
    // Amostra a profundidade media de um patch NxN pixels ao redor de (cx,
    // cy) na imagem de profundidade. Retorna NAN se nenhum pixel valido foi
    // encontrado na janela. Reaproveitado tanto para a leitura central
    // (LiDAR frontal) quanto para a varredura por setor (LiDAR 2D).
    static float samplePatchDepth(const gz::msgs::Image &msg, int cx, int cy, int patch_half);
    void laserScanCallback(const gz::msgs::LaserScan &msg);
    void opticalFlowCallback(const px4::msgs::OpticalFlow &msg);
    void magnetometerCallback(const gz::msgs::Magnetometer &msg);

    // Molde comum de pouso/desarme para IMU, motor e magnetometro (cada uma
    // com uma unica fonte de anomalia e uma unica fonte de posicao confiavel,
    // diferente do GPS, que tem blackout e anomalia como origens distintas).
    void checkGenericAnomalyAutoLand(
        uint64_t timestamp,
        bool anomaly_active,
        bool &landing_active,
        bool &auto_land_sent,
        uint64_t &auto_land_arrival_us,
        uint64_t &auto_land_last_arrived_us,
        double &land_hold_n_m,
        double &land_hold_e_m,
        const char *log_tag);
    void checkGenericAnomalyAutoDisarm(
        uint64_t timestamp,
        bool &landing_active,
        bool &auto_land_sent,
        bool &auto_disarm_sent,
        uint32_t &sim_agl_ground_count,
        const char *log_tag);
    void publishGenericLandCommand(uint64_t timestamp, double land_hold_n_m, double land_hold_e_m, const char *log_tag);
    void publishGenericDisarmCommand(uint64_t timestamp, bool force_disarm, const char *log_tag);

    // Pouso/desarme para a mitigacao de IMU: agora dedicado, dentro de
    // ImuMitigation (ver checkAutoLand/checkAutoDisarm em ImuMitigation.hpp).
    // void checkImuAutoLand(uint64_t timestamp);
    // void checkImuAutoDisarm(uint64_t timestamp);

    // Pouso/desarme para a mitigacao de motor (wrapper sobre o molde generico).
    // O gatilho vem da anomalia de atuacao detectada e ja corrigida em
    // GZMixingInterfaceESC::updateOutputs (consultada via
    // _mixing_interface_esc.motorAnomalyActive()).
    void checkMotorAutoLand(uint64_t timestamp);
    void checkMotorAutoDisarm(uint64_t timestamp);

    // Pouso/desarme para a mitigacao de magnetometro: agora dedicado, dentro
    // de MagnetometerMitigation (ver checkAutoLand/checkAutoDisarm).
    // void checkMagAutoLand(uint64_t timestamp);
    // void checkMagAutoDisarm(uint64_t timestamp);

    // Pouso/desarme para a mitigacao de LiDAR (wrapper sobre o molde generico).
    void checkLidarAutoLand(uint64_t timestamp);
    void checkLidarAutoDisarm(uint64_t timestamp);

    // Pouso/desarme para a mitigacao de LiDAR frontal (wrapper sobre o
    // molde generico).
    void checkLidarFrontAutoLand(uint64_t timestamp);
    void checkLidarFrontAutoDisarm(uint64_t timestamp);

    // Pouso/desarme para a mitigacao de LiDAR 2D (wrapper sobre o molde
    // generico).
    void checkLidar2dAutoLand(uint64_t timestamp);
    void checkLidar2dAutoDisarm(uint64_t timestamp);

    // Pouso/desarme para a mitigacao de barometro (wrapper sobre o molde
    // generico).
    void checkBaroAutoLand(uint64_t timestamp);
    void checkBaroAutoDisarm(uint64_t timestamp);

    static void rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU);

    static float generate_wgn();

    void addGpsNoise(double &latitude, double &longitude, double &altitude,
             float &vel_north, float &vel_east, float &vel_down);

    uORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Publication<distance_sensor_s>          _distance_sensor_pub{ORB_ID(distance_sensor)};
    // Instancia separada para o LiDAR frontal em modelos combinados (ex.:
    // x500_uavjamsim), que tem down e front simultaneamente com nomes de
    // link/sensor distintos. Sem isso, os dois publicariam na mesma
    // instancia unica de _distance_sensor_pub, um sobrescrevendo o outro.
    uORB::PublicationMulti<distance_sensor_s>     _distance_sensor_front_pub{ORB_ID(distance_sensor)};
    uORB::Publication<differential_pressure_s>    _differential_pressure_pub{ORB_ID(differential_pressure)};
    uORB::Publication<obstacle_distance_s>        _obstacle_distance_pub{ORB_ID(obstacle_distance)};
    uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_ground_truth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
    uORB::Publication<vehicle_attitude_s>         _attitude_ground_truth_pub{ORB_ID(vehicle_attitude_groundtruth)};
    uORB::Publication<vehicle_global_position_s>  _gpos_ground_truth_pub{ORB_ID(vehicle_global_position_groundtruth)};
    uORB::Publication<vehicle_local_position_s>   _lpos_ground_truth_pub{ORB_ID(vehicle_local_position_groundtruth)};
    uORB::PublicationMulti<sensor_gps_s>          _sensor_gps_pub{ORB_ID(sensor_gps)};
    uORB::PublicationMulti<sensor_baro_s>         _sensor_baro_pub{ORB_ID(sensor_baro)};
    uORB::PublicationMulti<sensor_accel_s>        _sensor_accel_pub{ORB_ID(sensor_accel)};
    uORB::PublicationMulti<sensor_gyro_s>         _sensor_gyro_pub{ORB_ID(sensor_gyro)};
    uORB::PublicationMulti<sensor_mag_s>          _sensor_mag_pub{ORB_ID(sensor_mag)};
    uORB::PublicationMulti<vehicle_odometry_s>    _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
    uORB::PublicationMulti<sensor_optical_flow_s> _optical_flow_pub{ORB_ID(sensor_optical_flow)};
    uORB::Publication<vehicle_command_s>          _vehicle_command_pub{ORB_ID(vehicle_command)};
    uORB::Subscription                            _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
    uORB::Subscription                            _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription                            _actuator_armed_sub{ORB_ID(actuator_armed)};

    GZMixingInterfaceESC   _mixing_interface_esc{_node};
    GZMixingInterfaceServo _mixing_interface_servo{_node};
    GZMixingInterfaceWheel _mixing_interface_wheel{_node};

    GZGimbal _gimbal{_node};

    MapProjection _pos_ref{};
    double _alt_ref{};

    matrix::Vector3d _position_prev{};
    matrix::Vector3d _velocity_prev{};
    matrix::Quatf _q_nb_prev{1.0f, 0.0f, 0.0f, 0.0f};  // quaternion NED<-body anterior, para calculo correto de velocidade angular
    hrt_abstime _timestamp_prev{};

    const std::string _world_name;
    const std::string _model_name;

    float _temperature{288.15};  // 15 degrees

    bool _realtime_clock_set{false};
    gz::transport::Node _node;

    // Ataques UAVJamSim
    AbsoluteOffsetAttack _gps_offset_attack{_node};
    RotationOffsetAttack _gps_rot_attack{_node};

    // Ataque de IMU
    ImuAttack _imu_attack{_node};

    // ======== MITIGACAO IMU - ESTADO ========
    // Substitui a linha de base, deteccao, cache de substituicao VIO e
    // referencia cruzada com GPS. Ver ImuMitigation.hpp/.cpp. O que fica
    // aqui (abaixo) e so a nota de que o pouso de IMU deixou de ser
    // compartilhado com motor/magnetometro/lidar/barometro - agora vive
    // inteiro dentro de ImuMitigation (ver checkAutoLand/checkAutoDisarm).
    ImuMitigation _imu_mitigation;

    // Pouso automatico mitigado por anomalia de IMU - movido para dentro de
    // ImuMitigation (nao mais compartilhado via checkGenericAnomalyAutoLand/Disarm).
    // bool     _imu_landing_active{false};
    // bool     _imu_auto_land_sent{false};
    // bool     _imu_auto_disarm_sent{false};
    // uint64_t _imu_auto_land_arrival_us{0};
    // uint64_t _imu_auto_land_last_arrived_us{0};
    // double   _imu_land_hold_n_m{0.0};
    // double   _imu_land_hold_e_m{0.0};
    // uint32_t _imu_sim_agl_ground_count{0};
    // ======== MITIGACAO IMU - FIM ========

    // ======== MITIGACAO MOTOR - ESTADO ========
    // A deteccao e a correcao em tempo real do comando de motor acontecem em
    // GZMixingInterfaceESC::updateOutputs (thread separada). Aqui so fica o
    // estado do pouso/desarme, disparado ao consultar motorAnomalyActive().
    bool     _motor_landing_active{false};
    bool     _motor_auto_land_sent{false};
    bool     _motor_auto_disarm_sent{false};
    uint64_t _motor_auto_land_arrival_us{0};
    uint64_t _motor_auto_land_last_arrived_us{0};
    double   _motor_land_hold_n_m{0.0};
    double   _motor_land_hold_e_m{0.0};
    uint32_t _motor_sim_agl_ground_count{0};
    // ======== MITIGACAO MOTOR - FIM ========

    // ======== MITIGACAO MAGNETOMETRO - ESTADO ========

    // Atitude da VIO (referencia independente do magnetometro), atualizada em
    // odometryCallback. So o heading (yaw) e usado no calculo; roll/pitch sao
    // usados apenas para exigir voo proximo do nivelado (ver
    // MAG_MAX_TILT_RAD), ja que a convencao de eixos do magnetometro simulado
    // nao e garantida como a mesma FRD usada por accel/gyro, entao nao
    // arriscamos compensar inclinacao via rotacao 3D. Compartilhada com as
    // mitigacoes de lidar front e lidar 2D, por isso continua aqui (nao foi
    // movida para dentro de MagnetometerMitigation).
    bool     _vio_heading_valid{false};
    uint64_t _vio_heading_us{0};
    float    _vio_heading_rad{0.0f};
    float    _vio_roll_rad{0.0f};
    float    _vio_pitch_rad{0.0f};
    // Inclinacao maxima (rad) para confiar no heading calculado a partir do
    // magnetometro sem compensacao de tilt. Duplicada dentro de
    // MagnetometerMitigation (mesmo valor) e tambem usada por lidar front/2D.
    static constexpr float MAG_MAX_TILT_RAD = 0.20f; // ~11.5 graus
    // Idade maxima aceita para a atitude da VIO usada como referencia.
    // Duplicada dentro de MagnetometerMitigation e tambem usada por lidar front/2D.
    static constexpr uint64_t MAG_VIO_MAX_AGE_US            = 100000ULL; // 100 ms

    // O restante do estado de deteccao/pouso de magnetometro foi movido para
    // dentro de MagnetometerMitigation (ver checkAutoLand/checkAutoDisarm).
    // bool     _mag_baseline_initialized{false};
    // uint32_t _mag_baseline_warmup_count{0};
    // double   _mag_heading_offset_rad{0.0};
    // static constexpr uint32_t MAG_BASELINE_WARMUP_SAMPLES = 60;
    // static constexpr double   MAG_BASELINE_WARMUP_ALPHA   = 0.25;
    // static constexpr double MAG_BASELINE_ALPHA = 0.01;

    // bool     _mag_anomaly_active{false};
    // uint64_t _mag_anomaly_start_us{0};
    // uint64_t _mag_recovery_start_us{0};
    // uint32_t _mag_mitig_diag_count{0};
    // static constexpr uint32_t MAG_MITIG_DIAG_MAX_COUNT = 5;

    // static constexpr double MAG_FIELD_HORIZ_MIN_G = 0.02;
    // static constexpr double MAG_FIELD_HORIZ_MAX_G = 1.0;
    // static constexpr double MAG_HEADING_ERROR_THRESHOLD_RAD = 0.35;   // ~20 graus
    // static constexpr double MAG_HEADING_ERROR_SEVERE_RAD    = 1.05;   // ~60 graus (nunca usada, ja era codigo morto)
    // static constexpr uint64_t MAG_ANOMALY_CONFIRM_US        = 3000000ULL; // 3 s
    // static constexpr uint64_t MAG_ANOMALY_CONFIRM_SEVERE_US = 20000ULL;  // 20 ms
    // static constexpr uint64_t MAG_RECOVERY_CONFIRM_US       = 500000ULL; // 500 ms

    // bool     _mag_landing_active{false};
    // bool     _mag_auto_land_sent{false};
    // bool     _mag_auto_disarm_sent{false};
    // uint64_t _mag_auto_land_arrival_us{0};
    // uint64_t _mag_auto_land_last_arrived_us{0};
    // double   _mag_land_hold_n_m{0.0};
    // double   _mag_land_hold_e_m{0.0};
    // uint32_t _mag_sim_agl_ground_count{0};
    MagnetometerMitigation _mag_mitigation;
    // ======== MITIGACAO MAGNETOMETRO - FIM ========

    // ======== MITIGACAO LIDAR - ESTADO ========

    // Ancora entre a leitura do LiDAR e a altitude do barometro, calibrada a
    // partir das primeiras amostras saudaveis (mesmo padrao de warmup do
    // magnetometro). Usa o barometro, nao a VIO, porque este airframe
    // (lidar_down) nao tem VIO/odometria visual confiavel disponivel — o
    // barometro e a unica referencia de altitude genuinamente independente
    // do LiDAR presente em todo airframe. O ataque de LiDAR e um offset
    // constante, que uma comparacao por taxa de variacao nunca pegaria (o
    // offset se cancela na diferenca entre duas amostras): so uma comparacao
    // por valor absoluto contra essa ancora detecta. Isso assume terreno
    // localmente plano perto do ponto de calibracao — sobrevoar uma
    // elevacao bem diferente da do momento da calibracao gera divergencia
    // legitima, nao deteccao de ataque.
    bool     _lidar_baseline_initialized{false};
    uint32_t _lidar_baseline_warmup_count{0};
    double   _lidar_anchor_offset_m{0.0}; // distancia_lida - altitude_baro, na calibracao
    static constexpr uint32_t LIDAR_BASELINE_WARMUP_SAMPLES = 30;
    static constexpr double   LIDAR_BASELINE_WARMUP_ALPHA   = 0.25;
    static constexpr double   LIDAR_BASELINE_ALPHA          = 0.01;

    bool     _lidar_anomaly_active{false};
    uint64_t _lidar_anomaly_start_us{0};
    uint64_t _lidar_recovery_start_us{0};
    uint32_t _lidar_mitig_diag_count{0};
    static constexpr uint32_t LIDAR_MITIG_DIAG_MAX_COUNT = 3;

    // Faixa fisica do LiDAR (LW20/C), igual ao configurado no SDF do
    // sensor (Tools/simulation/gz/models/x500_lidar_down|front/model.sdf).
    static constexpr float LIDAR_RANGE_MIN_M = 0.1f;
    static constexpr float LIDAR_RANGE_MAX_M = 100.0f;
    // Residuo maximo tolerado entre a leitura e o valor esperado pela ancora.
    static constexpr double LIDAR_RESIDUAL_THRESHOLD_M = 1.2;
    static constexpr uint64_t LIDAR_ANOMALY_CONFIRM_US        = 1500000ULL; // 1.5 s
    static constexpr uint64_t LIDAR_ANOMALY_CONFIRM_SEVERE_US = 20000ULL;   // 20 ms
    static constexpr uint64_t LIDAR_RECOVERY_CONFIRM_US       = 500000ULL;  // 500 ms
    static constexpr uint64_t LIDAR_BARO_MAX_AGE_US           = 800000ULL;  // 800 ms

    // Pouso automatico mitigado por anomalia de LiDAR.
    bool     _lidar_landing_active{false};
    bool     _lidar_auto_land_sent{false};
    bool     _lidar_auto_disarm_sent{false};
    uint64_t _lidar_auto_land_arrival_us{0};
    uint64_t _lidar_auto_land_last_arrived_us{0};
    double   _lidar_land_hold_n_m{0.0};
    double   _lidar_land_hold_e_m{0.0};
    uint32_t _lidar_sim_agl_ground_count{0};
    // ======== MITIGACAO LIDAR - FIM ========

    // ======== MITIGACAO LIDAR FRONTAL - ESTADO ========
    // Cache da leitura central da camera de profundidade (OakD-Lite, so
    // presente no x500_uavjamsim), usada como referencia independente do
    // LiDAR frontal. Diferente da mitigacao do LiDAR para baixo, aqui os
    // dois sensores medem a mesma coisa em tempo real (nao e um offset
    // constante que se cancela na diferenca entre amostras), entao nao
    // precisa de ancora/calibracao — e uma comparacao direta.
    bool     _depth_cam_distance_valid{false};
    uint64_t _depth_cam_distance_timestamp{0};
    float    _depth_cam_distance_m{NAN};
    static constexpr uint64_t LIDAR_FRONT_DEPTH_MAX_AGE_US = 300000ULL; // 300 ms

    bool     _lidar_front_anomaly_active{false};
    uint64_t _lidar_front_anomaly_start_us{0};
    uint64_t _lidar_front_recovery_start_us{0};
    uint32_t _lidar_front_mitig_diag_count{0};
    static constexpr uint32_t LIDAR_FRONT_MITIG_DIAG_MAX_COUNT = 3;

    // Residuo maximo tolerado entre LiDAR e camera de profundidade (folga
    // para o offset fisico/paralaxe entre os dois sensores no corpo).
    static constexpr double LIDAR_FRONT_RESIDUAL_THRESHOLD_M     = 1.0;
    // Inclinacao maxima para confiar na comparacao. Os dois sensores
    // apontam na mesma direcao (0 graus de diferenca angular pela geometria
    // do SDF), mas ficam a ~36cm de diferenca de altura no corpo (lidar
    // frontal bem baixo, camera bem mais alta, montada sobre o LiDAR 2D).
    // Em espaco aberto e nivelado, um raio horizontal nunca cruza o chao;
    // qualquer inclinacao residual, por menor que seja, faz o raio
    // eventualmente cruzar o chao a uma distancia que cresce conforme a
    // inclinacao diminui (relacao de tangente) — e a diferenca de altura
    // entre os sensores amplifica isso em divergencia real entre as duas
    // leituras, sem nenhum ataque envolvido. O limite aqui e bem mais
    // apertado que o do magnetometro (MAG_MAX_TILT_RAD) de proposito: por
    // essa relacao de tangente, mesmo uma inclinacao pequena ja e
    // suficiente para gerar residuo grande a media/longa distancia.
    static constexpr float LIDAR_FRONT_MAX_TILT_RAD = 0.05f; // ~3 graus
    static constexpr uint64_t LIDAR_FRONT_ANOMALY_CONFIRM_US        = 500000ULL; // 500 ms
    static constexpr uint64_t LIDAR_FRONT_ANOMALY_CONFIRM_SEVERE_US = 20000ULL;  // 20 ms
    static constexpr uint64_t LIDAR_FRONT_RECOVERY_CONFIRM_US       = 500000ULL; // 500 ms

    // Pouso automatico mitigado por anomalia de LiDAR frontal.
    bool     _lidar_front_landing_active{false};
    bool     _lidar_front_auto_land_sent{false};
    bool     _lidar_front_auto_disarm_sent{false};
    uint64_t _lidar_front_auto_land_arrival_us{0};
    uint64_t _lidar_front_auto_land_last_arrived_us{0};
    double   _lidar_front_land_hold_n_m{0.0};
    double   _lidar_front_land_hold_e_m{0.0};
    uint32_t _lidar_front_sim_agl_ground_count{0};
    // ======== MITIGACAO LIDAR FRONTAL - FIM ========

    // ======== MITIGACAO LIDAR 2D - ESTADO ========
    // Cache de profundidade por setor (mesma camera OakD-Lite do LiDAR
    // frontal, amostrada em varias colunas da imagem em vez de so o centro).
    // Cobre uma faixa de setores em torno da direcao frontal (indice
    // correspondente a 0 graus no referencial do corpo), usando o campo de
    // visao horizontal conhecido da camera para projetar cada angulo de
    // setor na coluna de pixel correspondente. So cobre uma fatia do scan
    // de 270 graus: fora dessa faixa nao ha nenhuma referencia independente
    // disponivel nesse modelo (mesma limitacao ja documentada no LiDAR
    // frontal), e a distancia de montagem entre lidar e camera introduz erro
    // de paralaxe que cresce em direcao as bordas do campo de visao — a
    // tolerancia de residuo (abaixo) e mais larga que a do LiDAR frontal por
    // causa disso.
    static constexpr int LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS = 7; // +-7 setores (~+-35 graus, teste)
    static constexpr int LIDAR2D_DEPTH_COVERAGE_COUNT = 2 * LIDAR2D_DEPTH_COVERAGE_HALF_SECTORS + 1;
    static constexpr uint64_t LIDAR2D_DEPTH_MAX_AGE_US = 300000ULL; // 300 ms

    float    _depth_cam_sector_distance_m[LIDAR2D_DEPTH_COVERAGE_COUNT] = {};
    bool     _depth_cam_sector_valid{false};
    uint64_t _depth_cam_sector_timestamp{0};

    bool     _lidar2d_anomaly_active{false};
    uint64_t _lidar2d_anomaly_start_us{0};
    uint64_t _lidar2d_recovery_start_us{0};
    uint32_t _lidar2d_mitig_diag_count{0};
    static constexpr uint32_t LIDAR2D_MITIG_DIAG_MAX_COUNT = 3;

    // Residuo medio maximo tolerado entre os setores do LiDAR 2D e a
    // referencia da camera (mais largo que o do LiDAR frontal por causa da
    // paralaxe crescente nas bordas da faixa coberta). Exige um numero
    // minimo de setores comparaveis para nao decidir com base em pouco dado.
    static constexpr double LIDAR2D_RESIDUAL_THRESHOLD_M = 1.5;
    // Mesma logica de inclinacao maxima do LiDAR frontal (ver comentario
    // detalhado la): a relacao de tangente entre inclinacao residual e
    // distancia ate o chao amplifica a diferenca de altura entre os
    // sensores, entao mesmo uma inclinacao pequena ja basta para gerar
    // residuo grande sem ataque nenhum — por isso o limite e apertado.
    static constexpr float LIDAR2D_MAX_TILT_RAD = 0.05f; // ~3 graus
    static constexpr int    LIDAR2D_MIN_VALID_SECTORS     = 3;
    static constexpr uint64_t LIDAR2D_ANOMALY_CONFIRM_US    = 500000ULL; // 500 ms
    static constexpr uint64_t LIDAR2D_RECOVERY_CONFIRM_US   = 500000ULL; // 500 ms

    // Pouso automatico mitigado por anomalia de LiDAR 2D.
    bool     _lidar2d_landing_active{false};
    bool     _lidar2d_auto_land_sent{false};
    bool     _lidar2d_auto_disarm_sent{false};
    uint64_t _lidar2d_auto_land_arrival_us{0};
    uint64_t _lidar2d_auto_land_last_arrived_us{0};
    double   _lidar2d_land_hold_n_m{0.0};
    double   _lidar2d_land_hold_e_m{0.0};
    uint32_t _lidar2d_sim_agl_ground_count{0};
    // ======== MITIGACAO LIDAR 2D - FIM ========

    // ======== MITIGACAO BAROMETRO - ESTADO ========
    // Ancora entre a altitude derivada do barometro e a altitude do GPS
    // (referencia independente), calibrada a partir das primeiras amostras
    // saudaveis. O ataque e um offset constante de altitude, que uma
    // comparacao por taxa de variacao nunca pegaria (o offset se cancela na
    // diferenca entre amostras): so uma comparacao por valor absoluto contra
    // essa ancora detecta.
    bool     _baro_baseline_initialized{false};
    uint32_t _baro_baseline_warmup_count{0};
    double   _baro_anchor_offset_m{0.0}; // altitude_barometro - altitude_gps, na calibracao
    static constexpr uint32_t BARO_BASELINE_WARMUP_SAMPLES = 30;
    static constexpr double   BARO_BASELINE_WARMUP_ALPHA   = 0.25;
    static constexpr double   BARO_BASELINE_ALPHA          = 0.01;

    bool     _baro_anomaly_active{false};
    uint64_t _baro_anomaly_start_us{0};
    uint64_t _baro_recovery_start_us{0};
    uint32_t _baro_mitig_diag_count{0};
    static constexpr uint32_t BARO_MITIG_DIAG_MAX_COUNT = 3;

    static constexpr double BARO_RESIDUAL_THRESHOLD_M = 3.0;
    static constexpr uint64_t BARO_ANOMALY_CONFIRM_US  = 1500000ULL; // 1.5 s
    static constexpr uint64_t BARO_RECOVERY_CONFIRM_US = 500000ULL;  // 500 ms
    static constexpr uint64_t BARO_GPS_ALT_MAX_AGE_US  = 1000000ULL; // 1 s

    // Pouso automatico mitigado por anomalia de barometro.
    bool     _baro_landing_active{false};
    bool     _baro_auto_land_sent{false};
    bool     _baro_auto_disarm_sent{false};
    uint64_t _baro_auto_land_arrival_us{0};
    uint64_t _baro_auto_land_last_arrived_us{0};
    double   _baro_land_hold_n_m{0.0};
    double   _baro_land_hold_e_m{0.0};
    uint32_t _baro_sim_agl_ground_count{0};
    // ======== MITIGACAO BAROMETRO - FIM ========

    // Ataque de magnetometro
    MagnetometerAttack _mag_attack{_node};

    // Ataque de lidar
    LidarAttack _lidar_attack{_node};

    // Ataque de stream
    int _stream_attack_option{0};
    gz::transport::Node::Publisher _stream_cmd_pub;

    // Ataque de barometro
    BarometerAttack _baro_attack{_node};

    // Ataques de jamming GPS
    JammingAttack _jamming_attack{_node};

    // Mitigacao GPS
    bool _attack_mitigation_enabled{false};

    // Mitigacao GPS - Filtro de Kalman com NIS

    // Ruido de processo do filtro
    static constexpr double JMIT_Q_POS        = 0.04;    // [m2/step]
    static constexpr double JMIT_Q_VEL        = 0.25;    // [(m/s)2/step]
    static constexpr double JMIT_Q_ALT        = 0.09;    // [m2/step]

    // Ruido nominal de medicao GPS
    static constexpr double JMIT_R_POS_CLEAN  = 0.64;    // [m2]   ~0.8 m
    static constexpr double JMIT_R_VEL_CLEAN  = 0.0025;  // [(m/s)2]
    static constexpr double JMIT_R_ALT_CLEAN  = 2.56;    // [m2]   ~1.6 m

    // Limiar NIS: 2(1-DOF, p=0.001) = 10.83.
    static constexpr double JMIT_NIS_THRESHOLD = 10.83;

    // Limites de velocidade aceitos para fontes auxiliares de dead-reckoning.
    static constexpr float JMIT_DR_VEL_MAX_M_S   = 15.0f;  // [m/s] horizontal
    static constexpr float JMIT_DR_VEL_Z_MAX_M_S =  5.0f;  // [m/s] vertical

    // Limites de sanidade da odometria visual usada como fonte auxiliar.
    static constexpr float VIO_MAX_POS_VAR_M2 = 25.0f;

    // Covariancia maxima da estimativa
    static constexpr float  JMIT_P_MAX         = 625.0f;  // [m2]   = 25 m

    // ======== MITIGACAO BLACKOUT GPS - CONSTANTES ========
    // Tempo maximo em que a mitigacao mantem a pseudo-medicao GPS durante blackout.
    static constexpr uint64_t BLACKOUT_MAX_HOLD_US = 300000000ULL; // 300 s

    // Qualidade degradada usada durante a pseudo-medicao GPS.
    static constexpr float BLACKOUT_EPH_DEGRADE  = 3.0f; // [m]
    static constexpr float BLACKOUT_EPV_DEGRADE  = 5.0f; // [m]
    static constexpr float BLACKOUT_HDOP_DEGRADE = 1.5f;
    static constexpr float BLACKOUT_VDOP_DEGRADE = 2.5f;

    // Frequencia de publicacao da pseudo-medicao durante blackout (10 Hz).
    static constexpr uint64_t BLACKOUT_PUB_INTERVAL_US = 100000ULL; // 0.1 s

    // Janela maxima para considerar a odometria visual recente durante blackout.
    static constexpr uint64_t BLACKOUT_VIO_TIMEOUT_US = 500000ULL; // 0.5 s

    // Tempo limite usado para identificar ausencia de atualizacoes GPS.
    static constexpr uint64_t BLACKOUT_GPS_TIMEOUT_US = 500000ULL; // 500 ms

    // ======== POUSO AUTOMATICO GPS - CONSTANTES ========
    static constexpr float AUTO_LAND_DEST_RADIUS_M = 1.2f;
    static constexpr float AUTO_LAND_MAX_SPEED_M_S = 0.25f;
    static constexpr float AUTO_LAND_ANOMALY_DEST_RADIUS_M = 1.2f;
    static constexpr float AUTO_LAND_ANOMALY_MAX_SPEED_M_S = 0.45f;
    static constexpr float AUTO_LAND_SEVERE_ANOMALY_DEST_RADIUS_M = 2.0f;
    static constexpr float AUTO_LAND_SEVERE_ANOMALY_MAX_SPEED_M_S = 0.45f;
    static constexpr uint64_t AUTO_LAND_STABLE_US = 1500000ULL;
    static constexpr uint64_t AUTO_LAND_PULSE_BRIDGE_US = 6000000ULL;
    static constexpr uint64_t AUTO_LAND_SEVERE_STABLE_US = 1500000ULL;
    static constexpr float AUTO_LAND_FINAL_MAX_SPEED_M_S = 0.40f;
    static constexpr uint64_t AUTO_LAND_FINAL_MAX_WAIT_US = 8000000ULL;
    static constexpr double AUTO_LAND_SEVERE_NIS_THRESHOLD = 50000.0;
    static constexpr uint64_t AUTO_LAND_MIN_BLACKOUT_US = 500000ULL;
    static constexpr uint64_t AUTO_LAND_MIN_NOISE_US = 3000000ULL;
    static constexpr double AUTO_LAND_DESCENT_RATE_M_S = 0.08;
    static constexpr double AUTO_LAND_TARGET_AGL_M = 0.30;
    static constexpr double AUTO_LAND_TARGET_GPS_ANOMALY_AGL_M = 0.25;
    static constexpr uint64_t AUTO_LAND_GROUND_DISARM_MIN_US = 0ULL;
    static constexpr uint64_t AUTO_LAND_GROUND_SENSOR_TIMEOUT_US = 1000000ULL;
    static constexpr float AUTO_LAND_GROUND_DISARM_DIST_M = 0.18f;
    static constexpr double AUTO_LAND_SIM_GROUND_DISARM_AGL_M = 0.12;
    static constexpr float AUTO_LAND_MAX_PITCH_RAD = 0.0872665f;
    static constexpr float AUTO_LAND_MAX_ROLL_RAD = 0.0872665f;
    static constexpr uint64_t AUTO_LAND_ATT_LOG_INTERVAL_US = 500000ULL;

    // Altitude barometrica auxiliar
    float _jmit_baro_alt_m{0.0f};
    bool _jmit_baro_valid{false};
    uint64_t _jmit_baro_timestamp{0};

    // Alinhamento entre altitude barometrica e altitude MSL
    double _jmit_baro_alt_offset{0.0};
    bool _jmit_baro_alt_offset_valid{false};

    // Ultima medicao GPS real publicada. A deteccao de blackout usa timeout deste cache.
    bool     _gps_real_valid{false};
    uint64_t _gps_real_last_us{0};
    double   _gps_real_n_m{0.0};
    double   _gps_real_e_m{0.0};
    double   _gps_real_alt{0.0};
    // Timestamp proprio da altitude: _gps_real_last_us marca toda chegada de
    // callback GPS (mesmo em ancora), mas _gps_real_alt so atualiza fora da
    // ancora — usar _gps_real_last_us para checar frescor da altitude
    // esconderia o caso de GPS sob ataque com a mitigacao do GPS ligada.
    uint64_t _gps_real_alt_timestamp{0};
    float    _gps_real_vel_n{0.0f};
    float    _gps_real_vel_e{0.0f};
    float    _gps_real_vel_d{0.0f};

    // Cache de odometria visual usado como fonte primaria durante blackout.
    bool     _vio_valid{false};
    uint64_t _vio_timestamp{0};
    double   _vio_n_m{0.0};
    double   _vio_e_m{0.0};
    double   _vio_d_m{0.0};
    double   _vio_alt_msl{0.0};
    float    _vio_vel_n{0.0f};
    float    _vio_vel_e{0.0f};
    float    _vio_vel_d{0.0f};

    // Posicao local e altitude MSL usadas no criterio de chegada ao destino.
    uORB::Subscription _lpos_ekf2_sub{ORB_ID(vehicle_local_position)};

    // Velocidade horizontal estimada usada no criterio de chegada ao destino.
    float _imu_dr_vel_n{0.0f};
    float _imu_dr_vel_e{0.0f};
    float _imu_dr_vel_d{0.0f};

    // Altitude local convertida para MSL para a pseudo-medicao GPS.
    double   _jmit_lpos_alt_msl{0.0};
    bool     _jmit_lpos_alt_valid{false};
    uint64_t _jmit_lpos_timestamp{0};

    // Posicao local usada para detectar chegada ao destino durante blackout.
    double   _lpos_n_m{0.0};
    double   _lpos_e_m{0.0};
    double   _lpos_alt_msl{0.0};
    bool     _lpos_xy_valid{false};
    bool     _lpos_z_valid{false};
    uint64_t _lpos_timestamp{0};
    float    _lpos_ground_speed{0.0f};
    float    _lpos_heading_rad{0.0f};   // Heading atual do drone [rad NED], atualizado via lpos.

    // Cache do sensor de distancia apontado para baixo.
    float    _ground_distance_m{NAN};
    bool     _ground_distance_valid{false};
    uint64_t _ground_distance_timestamp{0};

    double   _sim_agl_m{NAN};
    bool     _sim_agl_valid{false};
    uint64_t _sim_agl_timestamp{0};

    float    _sim_roll_rad{NAN};
    float    _sim_pitch_rad{NAN};
    bool     _sim_att_valid{false};
    uint64_t _sim_att_timestamp{0};

    static constexpr uint32_t SIM_AGL_GROUND_CONFIRM_COUNT = 5;

    // Substitui todo o estado do KF, blackout e pouso/desarme de GPS. Ver
    // GpsMitigation.hpp/.cpp (inclui GpsMitigationContext).
    GpsMitigation _gps_mitigation;

    // Modelo de ruido nominal do GPS
    float _gps_pos_noise_n = 0.0f;
    float _gps_pos_noise_e = 0.0f;
    float _gps_pos_noise_d = 0.0f;
    float _gps_vel_noise_n = 0.0f;
    float _gps_vel_noise_e = 0.0f;
    float _gps_vel_noise_d = 0.0f;
    const float _pos_noise_amplitude = 0.8f;    // [m]
    const float _pos_random_walk = 0.01f;
    const float _pos_markov_time = 0.95f;
    const float _vel_noise_amplitude = 0.05f;   // [m/s]
    const float _vel_noise_density = 0.2f;
    const float _vel_markov_time = 0.85f;

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used,
        (ParamInt<px4::params::SIM_GZ_EN_LIDAR>) _sim_gz_en_lidar,
        (ParamInt<px4::params::SIM_GZ_EN_FLOW>) _sim_gz_en_flow,
        (ParamInt<px4::params::SIM_GZ_EN_ASPD>) _sim_gz_en_aspd,
        (ParamInt<px4::params::SIM_GZ_EN_BARO>) _sim_gz_en_baro,
        (ParamInt<px4::params::SIM_GZ_EN_ODOM>) _sim_gz_en_odom,
        (ParamInt<px4::params::SIM_GZ_EN_GPS>) _sim_gz_en_gps
    )
};