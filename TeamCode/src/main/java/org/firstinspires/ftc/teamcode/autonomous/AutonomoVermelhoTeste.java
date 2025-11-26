package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadRunner.Drawing;
import org.firstinspires.ftc.teamcode.roadRunner.TankDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "AutonomoVermelhoTeste", group = "Autonomous")
public class AutonomoVermelhoTeste extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 24;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public static class SimplePIDController {
        private double kP, kI, kD;
        private double integralSum = 0;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public SimplePIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            timer.reset();
        }

        public double calculate(double reference, double state) {
            double error = reference - state;
            double dt = timer.seconds();

            if (dt <= 0) dt = 0.001;

            timer.reset();

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            return (error * kP) + (integralSum * kI) + (derivative * kD);
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            timer.reset();
        }
    }

    public static class Shooter {
        private CRServo shooterServo;
        private DcMotorEx shooterMotor;
        private DcMotorEx intakeMotor;

        public Shooter(HardwareMap hardwareMap) {
            shooterServo = hardwareMap.get(CRServo.class, "shooter_servo");
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public Action shoot() {
            return new ShooterOnAction();
        }

        public Action intakeOn() {
            return new IntakeOnAction();
        }

        public Action intakeOff() {
            return new IntakeOffAction();
        }

        public class ShooterOnAction implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooterMotor.setVelocity(1810);
                    intakeMotor.setPower(1);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double velocidadeAtual = Math.abs(shooterMotor.getVelocity());

                if (velocidadeAtual > 1760) {
                    shooterServo.setPower(-1);
                } else {
                    shooterServo.setPower(0);
                }

                if (timer.seconds() > 6.0) {
                    shooterMotor.setVelocity(0);
                    intakeMotor.setPower(0);
                    shooterServo.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public class IntakeOnAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(1);
                return false;
            }
        }

        public class IntakeOffAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(0);
                return false;
            }
        }
    }

    public class AutoAimAction implements Action {
        private final TankDrive drive;
        private SimplePIDController pid = new SimplePIDController(0.035, 0.0, 0.003);

        private static final double MAX_AUTO_TURN = 0.6;
        private static final double ALIGNMENT_THRESHOLD_DEGREES = 1.0;

        private ElapsedTime totalTimer;
        private boolean initialized = false;
        private final double timeLimitSeconds;

        public AutoAimAction(TankDrive drive, double timeLimitSeconds) {
            this.drive = drive;
            this.timeLimitSeconds = timeLimitSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                totalTimer = new ElapsedTime();
                pid.reset();
                initialized = true;
            }

            drive.updatePoseEstimate();

            if (totalTimer.seconds() >= timeLimitSeconds) {
                setDrivePowers(0, 0);
                return false;
            }

            AprilTagDetection detection = getAprilTagById(DESIRED_TAG_ID);

            if (detection == null) {
                setDrivePowers(0, 0);
                packet.put("AutoAim Status", "Procurando Tag...");
                return true;
            }

            double currentError = detection.ftcPose.bearing;

            if (Math.abs(currentError) < ALIGNMENT_THRESHOLD_DEGREES) {
                setDrivePowers(0, 0);
                packet.put("AutoAim Status", "ALINHADO");
                return false;
            }

            double turnPower = pid.calculate(0, currentError);

            double kF = 0.1;
            if (Math.abs(turnPower) < kF && Math.abs(currentError) > ALIGNMENT_THRESHOLD_DEGREES) {
                turnPower += Math.signum(turnPower) * kF;
            }

            turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double leftPower = turnPower;
            double rightPower = -turnPower;

            setDrivePowers(leftPower, rightPower);

            packet.put("AutoAim Erro (Graus)", currentError);
            packet.put("AutoAim Power", turnPower);

            return true;
        }

        private void setDrivePowers(double left, double right) {
            for (DcMotorEx m : drive.leftMotors) m.setPower(left);
            for (DcMotorEx m : drive.rightMotors) m.setPower(right);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        Shooter shooter = new Shooter(hardwareMap);

        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        Action autoAim = new AutoAimAction(drive, 3.0);

        Action trajectoryAction = drive.actionBuilder(initialPose)
                .lineToX(1)
                .stopAndAdd(autoAim)
                .stopAndAdd(shooter.shoot())
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(trajectoryAction);

        drive.updatePoseEstimate();

        if (visionPortal != null) visionPortal.close();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStroke("#3F51B5");
            Drawing.drawRobot(fieldOverlay, pose);

            packet.put("x", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("heading", Math.toDegrees(pose.heading.toDouble()));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
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