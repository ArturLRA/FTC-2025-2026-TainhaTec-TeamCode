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

@TeleOp(name="TeleOpRoboFinal", group="Linear OpMode")
public class TeleOpRoboFinal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotorEx shooterMotor = null;
    private CRServo shooterServo = null;

    private DcMotor intakeMotor = null;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double DRIVE_GAIN_YAW = 0.03;

    private static final double MAX_AUTO_TURN = 0.5;
    private static final int DESIRED_EXPOSURE_MS = 6;
    private static final int DESIRED_GAIN = 250;
    private double vel = 0;
    private double minShooterVel = 0;

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

        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                executeAutoAim();
            } else {
                motorPower();
                driveControlManual();
                shooterControlManual();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocidade Motor", shooterMotor.getVelocity());
            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }


    private void executeAutoAim() {
        AprilTagDetection detection = getPrimaryAprilTagDetection();

        if (detection != null) {
            double bearingError = detection.ftcPose.bearing;

            double turnPower = -bearingError * DRIVE_GAIN_YAW;
            turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double leftPower  = -turnPower;
            double rightPower = turnPower;

            if (Math.abs(turnPower) < 0.1 && Math.abs(bearingError) > 1.0) {
                double minPower = 0.15;
                if (turnPower > 0) {
                    leftPower = -minPower;
                    rightPower = minPower;
                } else {
                    leftPower = minPower;
                    rightPower = -minPower;
                }
            }

            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

            telemetry.addLine("--- MIRA AUTOMÁTICA ---");
            telemetry.addData("Tag ID", detection.id);
            telemetry.addData("Giro (Yaw)", "Erro: %.2f | Power: %.2f", bearingError, turnPower);

        } else {
            stopAllMotors();
            telemetry.addLine("--- MIRA AUTOMÁTICA ---");
            telemetry.addLine("Nenhuma Tag Visível!");
        }
    }

    private void stopAllMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        shooterMotor.setVelocity(0);
        shooterServo.setPower(0);
    }

    private void driveControlManual() {
        double drive = -gamepad1.left_stick_y;
        double turn  = -gamepad1.right_stick_x;

        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }

    private void motorPower() {
        if (gamepad1.a) {
            vel = 2100;
            minShooterVel = 2060;
        } else if (gamepad1.b){
            vel = 1850;
            minShooterVel = 1810;
        } else if (gamepad1.x) {
            vel = 0;
            minShooterVel = 0;
        }
    }

    private void shooterControlManual() {
        if (gamepad1.right_bumper) {
            shooterMotor.setVelocity(vel);
            intakeMotor.setPower(1);

            double velocidadeAtual = Math.abs(shooterMotor.getVelocity());

            if (vel > 0 && velocidadeAtual >= minShooterVel) {
                shooterServo.setPower(-1);
            } else {
                shooterServo.setPower(0);
            }

        } else {
            shooterMotor.setVelocity(0);
            shooterServo.setPower(0);
            intakeMotor.setPower(0);
        }

        if (gamepad1.left_bumper) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
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

    private AprilTagDetection getPrimaryAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (!currentDetections.isEmpty()) {
            return currentDetections.get(0);
        }
        return null;
    }
}