package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOpRoboAzul", group="Linear OpMode")
public class TeleOpRoboAzul extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Hardware
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotorEx shooterMotor = null;
    private CRServo shooterServo = null;
    private DcMotor intakeMotor = null;

    // Câmera e Vision
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double MAX_AUTO_TURN = 0.5;
    private static final int DESIRED_EXPOSURE_MS = 6;
    private static final int DESIRED_GAIN = 250;

    private double targetShooterVel = 0;
    private double minShooterVelThreshold = 0;

    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private static final double KP = 0.035;
    private static final double KD = 0.0005;
    private static final double KF = 0.20;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Iniciando Hardware...");
        telemetry.update();

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");
        shooterMotor    = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterServo    = hardwareMap.get(CRServo.class, "shooter_servo");
        intakeMotor     = hardwareMap.get(DcMotor.class, "intake_motor");

        configureDriveMotors();

        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(DESIRED_EXPOSURE_MS, DESIRED_GAIN);
        }

        telemetry.addData("Status", "Pronto para Start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad2.y) {
                executeAutoAim();
            } else {
                driveControlManual();
            }

            intakeControl();

            shooterLogic();

            telemetry.addData("Status", "Tempo: " + runtime.toString());
            telemetry.addData("Shooter Alvo", targetShooterVel);
            telemetry.addData("Shooter Atual", shooterMotor.getVelocity());
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void driveControlManual() {
        double drive = -gamepad1.right_stick_y;
        double turn  = gamepad1.left_stick_x;

        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }

    private void intakeControl() {
        double intakePower;

        if (gamepad1.right_trigger > 0.1) {
            intakePower = -1.0;
        } else if (gamepad1.left_trigger > 0.1) {
            intakePower = 1.0;
        } else {
            intakePower = 0.25;
        }

        intakeMotor.setPower(intakePower);
    }

    private void shooterLogic() {
        if (gamepad2.a) {
            targetShooterVel = 2000;
            minShooterVelThreshold = 1960;
        } else if (gamepad2.b) {
            targetShooterVel = 1750;
            minShooterVelThreshold = 1710;
        } else if (gamepad2.x) {
            targetShooterVel = 0;
            minShooterVelThreshold = 0;
        }

        if (gamepad2.dpad_down) {
            shooterMotor.setVelocity(-1000);
            shooterServo.setPower(-1);
        } else {
            shooterMotor.setVelocity(targetShooterVel);

            if (gamepad2.right_trigger > 0.1) {
                double currentVel = Math.abs(shooterMotor.getVelocity());

                if (targetShooterVel > 0 && currentVel >= minShooterVelThreshold) {
                    shooterServo.setPower(-1);
                } else {
                    shooterServo.setPower(0);
                }
            } else {
                shooterServo.setPower(0);
            }
        }
    }

    private void executeAutoAim() {
        AprilTagDetection detection = getAprilTagById(24);

        if (detection != null) {
            double currentError = detection.ftcPose.bearing;

            double currentTime = pidTimer.seconds();
            if (currentTime == 0) currentTime = 0.001;

            double derivative = (currentError - lastError) / currentTime;
            pidTimer.reset();
            lastError = currentError;

            double pTerm = currentError * KP;
            double dTerm = derivative * KD;
            double fTerm = Math.signum(currentError) * KF;

            double turnPower = -(pTerm + dTerm + fTerm);

            if (Math.abs(currentError) < 0.5) {
                turnPower = 0;
                lastError = 0;
            }

            turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double leftPower  = -turnPower;
            double rightPower = turnPower;

            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

            telemetry.addLine("--- MIRA PIDF ---");
            telemetry.addData("Erro", "%.3f", currentError);
        } else {
            stopAllMotors();
            lastError = 0;
            telemetry.addLine("--- MIRA ---");
            telemetry.addLine("Tag não encontrada!");
        }
    }

    private void stopAllMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        shooterMotor.setVelocity(0);
        shooterServo.setPower(0);
        intakeMotor.setPower(0.25);
    }

    private void configureDriveMotors() {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
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

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null || !USE_WEBCAM) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
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
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private AprilTagDetection getAprilTagById(int targetId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }
}