/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <regex>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <opencv2/opencv.hpp>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include "StreamAttack.hpp"

namespace custom
{
class GstCameraSystem :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPostUpdate
{
public:
	GstCameraSystem();
	~GstCameraSystem();

	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PostUpdate(const gz::sim::UpdateInfo &_info,
			const gz::sim::EntityComponentManager &_ecm) override;

private:
	void onImage(const gz::msgs::Image &msg);
	void onCameraInfo(const gz::msgs::Image &msg);

	// Find first camera topic in the world
	void findCameraTopic();

	void gstThreadFunc();

	// Transport
	gz::transport::Node _node;

	// Image processing
	gz::msgs::Image _currentFrame;
	std::mutex _frameMutex;
	std::atomic<bool> _newFrameAvailable {};

	// GStreamer elements
	GMainLoop *_gstLoop {};
	GstElement *_pipeline {};
	GstElement *_source {};
	std::thread _gstThread;
	std::atomic<bool> _running {};

	// Configuration
	std::string _worldName;
	std::string _udpHost;
	int _udpPort = 5600;
	bool _useRtmp {};
	std::string _rtmpLocation;
	bool _useCuda = true;

	// Topic info
	std::string _cameraTopic;
	int _width {};
	int _height {};
	double _rate = 30.0;

	// Topic pattern for matching camera image topics
	std::regex _cameraTopicPattern;

	// Flag to control discovery
	bool _initialized {};

	// ataque stream
	StreamAttack _streamAttack{_node};

	// ======== MITIGACAO GERAL - INICIO ========
	// Mesmo topico unico de liga/desliga que todas as mitigacoes do
	// GZBridge ja respeitam (ligado via ./attack_mitigator --on/--off).
	void onMitigationCmd(const gz::msgs::Vector3d &msg);
	std::atomic<bool> _mitigationEnabled{false};
	// ======== MITIGACAO GERAL - FIM ========

	// ======== MITIGACAO FLIP - INICIO ========
	// Deteccao por continuidade temporal, sem consultar _stream_attack_option:
	// compara o frame atual contra o ultimo frame bom guardado, nas duas
	// formas (como esta, e invertido verticalmente). Um flip de verdade so
	// deixa de divergir do historico quando desfeito; movimento normal de
	// camera nunca produz essa assinatura. So roda enquanto _mitigationEnabled.
	cv::Mat _lastGoodFrameGray;
	int _flipAnomalyStreak{0};
	int _flipRecoveryStreak{0};
	bool _flipAnomalyActive{false};
	// Publica um sinal quando a anomalia e confirmada, para o GZBridge (que
	// roda no processo do PX4, com acesso ao terminal via PX4_WARN) avisar
	// o operador — este plugin roda no processo do Gazebo, sem ligacao
	// nenhuma com esse terminal.
	gz::transport::Node::Publisher _flipDetectedPub;
	static constexpr int FLIP_DETECT_WIDTH = 160;
	static constexpr int FLIP_DETECT_HEIGHT = 90;
	static constexpr int FLIP_CONFIRM_FRAMES = 3;
	static constexpr int FLIP_RECOVERY_FRAMES = 3;
	// Diferenca minima absoluta (em niveis de cinza, 0-255) para considerar
	// o quadro atual "genuinamente diferente" do ultimo bom; abaixo disso,
	// nao ha dado suficiente para decidir (cena praticamente parada).
	static constexpr double FLIP_MIN_ABS_DIFF = 8.0;
	// A comparacao invertida precisa bater pelo menos essa fracao melhor
	// que a comparacao normal para contar como sintoma.
	static constexpr double FLIP_MATCH_RATIO = 0.6;
	// ======== MITIGACAO FLIP - FIM ========

	// ======== MITIGACAO QUADRADO PRETO - INICIO ========
	// Deteccao radial a partir do centro do frame, usando o conhecimento
	// parcial de que o ataque sempre fica centralizado (sem assumir
	// tamanho exato): a partir do pixel central, anda em 8 direcoes
	// (cima, baixo, esquerda, direita, 4 diagonais) contando quantos
	// pixels quase-pretos consecutivos existem antes de bater em algo que
	// nao e escuro. So conta como sintoma se o centro for escuro e a
	// extensao escura alcancar um raio minimo em praticamente todas as
	// direcoes ao mesmo tempo — um bloco solido de ataque se expande de
	// forma uniforme a partir do centro; uma arvore, mesmo escura, quase
	// sempre tem contorno irregular (galhos, aberturas entre folhas) que
	// quebra a continuidade em pelo menos algumas direcoes antes de
	// alcancar esse raio, mesmo quando calha de estar bem no centro do
	// quadro.
	int _blackAnomalyStreak{0};
	int _blackRecoveryStreak{0};
	bool _blackAnomalyActive{false};
	static constexpr int BLACK_CONFIRM_FRAMES = 5;
	static constexpr int BLACK_RECOVERY_FRAMES = 3;
	// Pixels de cinza abaixo deste valor (0-255) sao considerados escuros.
	static constexpr int BLACK_PIXEL_THRESHOLD = 15;
	// Raio minimo (pixels) que a extensao escura precisa alcancar em cada
	// direcao a partir do centro. Ataques menores que isso nao sao
	// detectados por esta checagem. O ataque real usa lado igual a
	// min(largura,altura)/6 (ver squareSize na aplicacao do ataque, mais
	// abaixo nesta funcao) — em 1920x1080 isso da 180px de lado, ou seja,
	// so 90px do centro ate a borda em qualquer direcao. O valor abaixo
	// fica com folga confortavel abaixo desse maximo fisico.
	static constexpr int BLACK_MIN_RADIUS_PX = 60;
	// Das 8 direcoes testadas, quantas precisam alcancar o raio minimo
	// para confirmar o sintoma. Nao exige as 8 (tolera alguma quebra de
	// continuidade nas bordas, por exemplo se o ataque estiver fundido a
	// outro objeto de um dos lados) sem abrir mao da maioria delas.
	static constexpr int BLACK_MIN_DIRECTIONS_OK = 7;
	// Mesmo mecanismo do flip: avisa o GZBridge (processo do PX4, com
	// acesso ao terminal via PX4_WARN) quando a anomalia e confirmada.
	gz::transport::Node::Publisher _blackDetectedPub;
	// ======== MITIGACAO QUADRADO PRETO - FIM ========

};
}  // namespace custom