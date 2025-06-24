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

@Autonomous(name = "Teste_Pose_Estimada", group = "Testes")
public class teste extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, startPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Inicie para ver a pose estimada no Dashboard.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            Pose2d pose = drive.getPoseEstimate();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();

            c.setStroke("#000000");
            c.strokeRect(-72, -72, 144, 144);

            c.setFill("#FF9800");
            c.fillCircle(pose.position.x, pose.position.y, 3);
            c.strokeLine(
                    pose.position.x,
                    pose.position.y,
                    pose.position.x + Math.cos(pose.heading.toDouble()) * 6,
                    pose.position.y + Math.sin(pose.heading.toDouble()) * 6
            );

            packet.put("x", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}
