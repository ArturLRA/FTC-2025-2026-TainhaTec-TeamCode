package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.Drawing;
import org.firstinspires.ftc.teamcode.roadRunner.TankDrive;

@Config
@Autonomous(name = "AutonomoRR01", group = "Autonomous")
public class AutonomoRR01 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(30)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90));

        Action trajectoryAction = tab1.build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Aguardando start");
            telemetry.addData("Posição detectada", visionOutputPosition);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Executando trajetória");
        telemetry.update();

        Actions.runBlocking(trajectoryAction);

        drive.updatePoseEstimate();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            Canvas fieldOverlay = packet.fieldOverlay();

            fieldOverlay.setStroke("#3F51B5");
            Drawing.drawRobot(fieldOverlay, pose);

            double[] xPoints = new double[drive.poseHistory.size()];
            double[] yPoints = new double[drive.poseHistory.size()];
            for (int i = 0; i < drive.poseHistory.size(); i++) {
                xPoints[i] = drive.poseHistory.get(i).position.x;
                yPoints[i] = drive.poseHistory.get(i).position.y;
            }
            fieldOverlay.setStroke("#4CAF50");
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.strokePolyline(xPoints, yPoints);

            packet.put("x", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            telemetry.addData("Pose", pose);
            telemetry.update();

            com.acmerobotics.dashboard.FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}







