package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;
import android.graphics.Canvas;
import android.opengl.GLSurfaceView;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl; // Importar
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;     // Importar
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOpCameraMod", group = "Linear OpMode")
public class TeleOpCameraMod extends LinearOpMode{

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int myExposure;
    private int minExposure;
    private int maxExposure;
    private int myGain;
    private int minGain;
    private int maxGain;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;

    @Override
    public void runOpMode() {

        initAprilTag();

        if (USE_WEBCAM) {
            getCameraSetting();
            myExposure = Range.clip(5, minExposure, maxExposure);
            myGain = Range.clip(250, minGain, maxGain);
            setManualExposure(myExposure, myGain);
        }

        telemetry.addData("Camera", "Preview no DS + Dashboard");
        telemetry.addData("Exposição", "Ajuste com Left Bumper/Trigger");
        telemetry.addData("Ganho", "Ajuste com Right Bumper/Trigger");
        telemetry.addData(">", "Press START to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleExposureAndGainControls();

            telemetryAprilTag();

            telemetry.addLine("---");
            telemetry.addData("Exposição (ms)", "%d (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Ganho", "%d (%d - %d)", myGain, minGain, maxGain);
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();

    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setLensIntrinsics(529.702, 529.702, 285.136, 307.352)
                .build();

    VisionPortal.Builder builder = new VisionPortal.Builder();

    if (USE_WEBCAM) {
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new android.util.Size(640,480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    }

    builder.enableLiveView(true);
    builder.setAutoStopLiveView(false);
    visionPortal = builder.build();
    FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detectadas", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(">> TAG ID %d (%s)", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("   Distância: %.2f cm", detection.ftcPose.range));
                telemetry.addLine(String.format("   Guinada: %.2f graus", detection.ftcPose.yaw));

            } else {
                telemetry.addLine(String.format(">> TAG ID %d (Desconhecida)", detection.id));
                telemetry.addLine("   (Não foi possível calcular a pose)");
            }
        }
    }

    private void handleExposureAndGainControls() {
        thisExpUp = gamepad1.left_bumper;
        thisExpDn = gamepad1.left_trigger > 0.25;
        thisGainUp = gamepad1.right_bumper;
        thisGainDn = gamepad1.right_trigger > 0.25;

        if (thisExpUp && !lastExpUp) {
            myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
            setManualExposure(myExposure, myGain);
        } else if (thisExpDn && !lastExpDn) {
            myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
            setManualExposure(myExposure, myGain);
        }

        if (thisGainUp && !lastGainUp) {
            myGain = Range.clip(myGain + 1, minGain, maxGain);
            setManualExposure(myExposure,myGain);
        } else if (thisGainDn && !lastGainDn) {
            myGain = Range.clip(myGain - 1, minGain, maxGain);
            setManualExposure(myExposure, myGain);
        }

        lastExpUp = thisExpUp;
        lastExpDn = thisExpDn;
        lastGainUp = thisGainUp;
        lastGainDn = thisGainDn;
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return false;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if (opModeInInit()) {
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
            } else {
                return false;
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return true;
        } else {
            return false;
        }
    }

    private void getCameraSetting() {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if (opModeInInit()) {
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
            } else {
                return;
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }

}
