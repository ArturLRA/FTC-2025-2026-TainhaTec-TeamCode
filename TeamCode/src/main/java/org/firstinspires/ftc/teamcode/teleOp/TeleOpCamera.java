package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "AjustableCamera", group = "Linear OpMode")

public class TeleOpCamera extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();
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

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detectadas", currentDetections.size());
        telemetry.addLine("---");

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(">> TAG ID %d (%s)", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("   Distância: %.2f cm", detection.ftcPose.range));
                telemetry.addLine(String.format("   Rolagem: %.2f graus", detection.ftcPose.roll));
                telemetry.addLine(String.format("   Inclinação: %.2f graus", detection.ftcPose.pitch));
                telemetry.addLine(String.format("   Guinada: %.2f graus", detection.ftcPose.yaw));
            } else {
                telemetry.addLine(String.format(">> TAG ID %d (Desconhecida)", detection.id));
                telemetry.addLine("   (Não foi possível calcular a distância)");
            }
        }
    }
}