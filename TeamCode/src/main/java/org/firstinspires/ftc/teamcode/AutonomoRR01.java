package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "AutonomoRR01", group = "Autonomous")
public class AutonomoRR01 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Pose inicial: centro da arena, voltado para frente (X positivo)
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Inicializa o TankDrive e Dashboard
        TankDrive drive = new TankDrive(hardwareMap, startPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Espera o início
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Aguardando início...");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Define uma sequência simples de movimentos
        Action routine = new SequentialAction(
                drive.actionBuilder(startPose)
                        .lineToX(24)
                        .waitSeconds(1)
                        .lineToX(48)
                        .build()
        );

        // Inicia a execução de forma assíncrona
        drive.runAsync(routine);

        // Loop de execução e envio para o Dashboard
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();

            Pose2d pose = drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            // Desenha a arena (12 ft x 12 ft = 144 x 144 in)
            fieldOverlay.setStroke("#000000");
            fieldOverlay.strokeRect(-72, -72, 144, 144);

            // Desenha o robô (círculo + direção)
            fieldOverlay.setFill("#4CAF50");
            fieldOverlay.fillCircle(pose.position.x, pose.position.y, 3);
            fieldOverlay.strokeLine(
                    pose.position.x,
                    pose.position.y,
                    pose.position.x + Math.cos(pose.heading.toDouble()) * 6,
                    pose.position.y + Math.sin(pose.heading.toDouble()) * 6
            );

            dashboard.sendTelemetryPacket(packet);
        }

        // Exibe a pose final no Driver Station
        Pose2d finalPose = drive.getPoseEstimate();
        telemetry.addData("x", finalPose.position.x);
        telemetry.addData("y", finalPose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();
    }
}
