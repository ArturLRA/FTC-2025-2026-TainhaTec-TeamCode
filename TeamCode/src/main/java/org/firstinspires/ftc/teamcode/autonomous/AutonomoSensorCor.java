package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.Drawing;
import org.firstinspires.ftc.teamcode.roadRunner.TankDrive;

@Config
@Autonomous(name = "AutonomoSensorCor", group = "Autonomous")
public class AutonomoSensorCor extends LinearOpMode {

    public enum SampleColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private TankDrive drive;
    private CRServo servoGarra;
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private final double CLAW_POWER = 1.0;
    private final double PICKUP_DISTANCE_INCHES = 5.0;
    private final double COLOR_CONFIDENCE_THRESHOLD = 0.5;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        servoGarra = hardwareMap.get(CRServo.class, "servo_garra");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor.setGain(15);

        int visionOutputPosition = 1;

        Action trajectoryAction = drive.actionBuilder(initialPose)
                .lineToX(30)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Executando trajetória");
        telemetry.update();

        closeClaw();
        sleep(1000);

        if (isSampleDetected()) {
            SampleColor detectedColor = getSampleColor();
            sleep(1000);

            switch (detectedColor) {
                case RED:
                case YELLOW:
                    openClaw();
                    sleep(1000);
                    break;
                case UNKNOWN:
                    openClaw();
                    sleep(1000);
                    break;
            }
        }

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

    private void closeClaw() {
        servoGarra.setPower(1);
    }

    private void openClaw() {
        servoGarra.setPower(-1);
    }

    private void stopClaw() {
        servoGarra.setPower(0);
    }

    private boolean isSampleDetected() {
        return distanceSensor.getDistance(DistanceUnit.INCH) < PICKUP_DISTANCE_INCHES;
    }

    private SampleColor getSampleColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.red > colors.blue && colors.red > colors.green && colors.red > COLOR_CONFIDENCE_THRESHOLD) {
            return SampleColor.RED;
        } else if (colors.blue > colors.red && colors.blue > colors.green && colors.blue > COLOR_CONFIDENCE_THRESHOLD) {
            return SampleColor.BLUE;
        } else if (colors.red > 0.2 && colors.green > 0.2 && colors.blue < 0.1) {
            return SampleColor.YELLOW;
        }

        return SampleColor.UNKNOWN;
    }

}