# TainhaTec - FTC TeamCode (2025-2026)

![Status](https://img.shields.io/badge/Status-Finalizado-blue)
![Java](https://img.shields.io/badge/Language-Java-orange)
![RoadRunner](https://img.shields.io/badge/Library-RoadRunner_v1.0-red)

Repositório oficial de software da equipa **TainhaTec** para a temporada 2025-2026 da *FIRST* Tech Challenge (FTC). Este projeto implementa algoritmos avançados de controle, visão computacional e automação para um robô de alto desempenho.

## 🚀 Visão Geral

O projeto foi estruturado para fornecer máxima precisão tanto no período autónomo quanto no TeleOperado. Utilizamos a biblioteca **Road Runner v1.0** para odometria e trajetórias baseadas em splines, integrada a um sistema de mira automática via **AprilTags**.

### Principais Funcionalidades:
* **Controle de Trajetória:** Implementação de cinemática para chassis `MecanumDrive` e `TankDrive` utilizando Road Runner para movimentação precisa no campo.
* **Visão Computacional:** Sistema de deteção de AprilTags para localização e mira automática (Auto-Aim).
* **Controle PIDF:** Algoritmo de controle para estabilização de mira (com ganhos Proporcional e Derivativo) e controlo de velocidade do Shooter.
* **Dashboard Integration:** Telemetria avançada e ajuste de parâmetros em tempo real via FTC Dashboard.

## 📂 Estrutura do Projeto

* [`TeamCode/autonomous`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous): Scripts de autonomia baseados em ações sequenciais e lógica de mira.
* [`TeamCode/teleOp`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleOp): Modos de operação manual com suporte a mira assistida e controle de periféricos como Intake e Shooter.
* [`TeamCode/roadRunner`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/roadRunner): Configurações de hardware, cinemática e classes de localização (Odometria).

## 🛠️ Detalhes Técnicos

### Sistema de Mira (Auto-Aim)
O robô utiliza o processador `AprilTagProcessor` para calcular o *bearing* (ângulo relativo) em relação ao alvo (ID 20 ou 24). No modo `TeleOpRoboAzul`, um controle **PIDF** é aplicado para alinhar o shooter com erro inferior a 0.5 graus.

### Mecanismos de Disparo
O sistema `Shooter` opera com controle de velocidade por encoders, garantindo que o servo de disparo (`shooter_servo`) só seja ativado quando o motor atingir a velocidade alvo (ex: 2000 RPM), minimizando erros de consistência.

## ⚙️ Configuração e Instalação

1.  Clone este repositório no seu Android Studio.
2.  Certifique-se de que o `HardwareMap` no robô corresponde aos nomes definidos em `MecanumDrive.java` e `TeleOpRoboAzul.java`.
3.  Aceda às [instruções do Road Runner](https://rr.brott.dev/docs/v1-0/tuning/) para calibrar parâmetros como `inPerTick` e `trackWidthTicks`.

## 🏆 Sobre a Equipe

A **TainhaTec** dedica-se à inovação técnica e ao desenvolvimento de competências em robótica. Este código reflete o compromisso da equipe com a engenharia de software de alta qualidade, focando-se em modularidade, documentação rigorosa e na busca por resultados de destaque no cenário competitivo da FTC.

---
## 👥 Desenvolvedores

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/arturlra">
        <img src="https://github.com/arturlra.png" width="100px;" alt="Foto de Artur Luiz"/><br />
        <sub><b>Artur Luiz Rodrigues Alves</b></sub>
      </a><br />
      Lider do Projeto
    </td>
    <td align="center">
      <a href="#">
        <img src="https://via.placeholder.com/100" width="100px;" alt="Foto de Ismael"/><br />
        <sub><b>Ismael</b></sub>
      </a><br />
      Desenvolvedor
    </td>
  </tr>
</table>

---
**Equipa TainhaTec** 🇧🇷
