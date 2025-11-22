package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.concurrent.TimeUnit;

import java.util.List;

@TeleOp(name = "TeleOpCamera", group = "Linear OpMode")

public class TeleOpCamera extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor coreHexMotor = null;

    private static final double GAIN_YAW = 0.02;
    private static final double GAIN_PITCH_SECONDARY = 0.005;
    private static final double MAX_POWER = 0.5;
    private static final int DESIRED_EXPOSURE_MS = 6;
    private static final int DESIRED_GAIN = 250;

    @Override
    public void runOpMode() {

        coreHexMotor = hardwareMap.get(DcMotor.class, "core_hex_motor");
        coreHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        coreHexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        if (USE_WEBCAM) {
            setManualExposure(DESIRED_EXPOSURE_MS, DESIRED_GAIN);
        }

        waitForStart();

        while (opModeIsActive()) {

            AprilTagDetection primaryDetection = getPrimaryAprilTagDetection();
            double motorPower = 0.0;

            if (primaryDetection != null) {

                double yawError = primaryDetection.ftcPose.yaw;
                double pitch = primaryDetection.ftcPose.pitch;
                double roll = primaryDetection.ftcPose.roll;

                double yawCorrection = yawError * GAIN_YAW;

                double rollInfluence = 1.0 - (Math.abs(roll) / 45.0);
                rollInfluence = Range.clip(rollInfluence, 0.3, 1.0);

                double pitchCorrection = pitch * GAIN_PITCH_SECONDARY;

                motorPower = (yawCorrection * rollInfluence) + pitchCorrection;

                motorPower = Range.clip(motorPower, -MAX_POWER, MAX_POWER);
                coreHexMotor.setPower(motorPower);

                telemetryAprilTag(primaryDetection);
                telemetry.addData("Motor Core Hex Power", "%.3f", motorPower);
                telemetry.addData("Yaw Corretion", "%.3f", yawCorrection);
                telemetry.addData("Roll Influence (Fator)", "%.2f", rollInfluence);

            } else {
                coreHexMotor.setPower(0.0);
                telemetry.addLine(">> Nenhuma AprilTag detectada. Motor Parado.");
            }

            telemetry.update();

            sleep(20);

        }

        visionPortal.close();
    }

    private AprilTagDetection getPrimaryAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (!currentDetections.isEmpty()) {
            return currentDetections.get(0);
        }
        return null;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(529.702, 529.702, 285.136, 307.352)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new android.util.Size(640, 480));
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        }

        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    private void telemetryAprilTag(AprilTagDetection detection) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detectadas", currentDetections.size());
        telemetry.addLine("---");

        if (detection.metadata != null) {
            telemetry.addLine(String.format(">> TAG ID %d (%s)", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("   Distância: %.2f cm", detection.ftcPose.range));
            telemetry.addLine(String.format("   Inclinação (Pitch): %.2f graus", detection.ftcPose.pitch));
            telemetry.addLine(String.format("   Rolagem (Roll): %.2f graus", detection.ftcPose.roll));
            telemetry.addLine(String.format("   Guinada (Yaw): %.2f graus", detection.ftcPose.yaw));
        } else {
            telemetry.addLine(String.format(">> TAG ID %d (Desconhecida)", detection.id));
            telemetry.addLine("   (Não foi possível calcular a distância)");
        }
    }

    private void setManualExposure (int exposureMS, int gain) {
        if (visionPortal == null || !USE_WEBCAM) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Aguardando Streaming");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addLine("Controle de Exposição e Ganho manual aplicado.");
            telemetry.update();
        }
    }
}