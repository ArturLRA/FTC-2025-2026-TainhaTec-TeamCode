package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// Importações Road Runner
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Importações FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Importação do drive personalizado
import org.firstinspires.ftc.teamcode.TankDrive;

@Config
@Autonomous(name = "AutonomoRR01", group = "Autonomous")
public class AutonomoRR01 extends LinearOpMode {

    // Classe Claw com servo da garra e ações integradas
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "servo_claw");
        }

        // Ação de fechar a garra
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55); // Fecha a garra
                return false; // Ação executada uma vez só
            }
        }

        // Ação de abrir a garra
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0); // Abre a garra
                return false;
            }
        }

        // Métodos auxiliares para retornar as ações
        public Action closeClaw() {
            return new CloseClaw();
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Define a posição inicial do robô
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Instancia os subsistemas
        TankDrive drive = new TankDrive(hardwareMap, startPose);
        Claw claw = new Claw(hardwareMap);

        // Fecha a garra inicialmente
        Actions.runBlocking(claw.closeClaw());

        // Aguarda o início do modo autônomo
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Waiting for start");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Trajetória inicial: sobe em Y e vai para X=12
        Action routine = drive.actionBuilder(startPose)
                .lineToY(24)
                .waitSeconds(1)
                .lineToX(12)
                .build();

        // Trajetória final de saída
        Action exitRoutine = drive.actionBuilder(new Pose2d(12, 24, 0))
                .lineToX(48)
                .build();

        // Executa toda a sequência de forma bloqueante
        Actions.runBlocking(new SequentialAction(
                routine,
                claw.openClaw(),
                exitRoutine
        ));
    }
}
