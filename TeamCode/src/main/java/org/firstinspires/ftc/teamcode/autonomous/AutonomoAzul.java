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
@Autonomous(name = "AutonomoAzul", group = "Autonomous")
public class AutonomoAzul extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 20;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public class Shooter {
        private CRServo shooterServo;
        private DcMotorEx shooterMotor;
        private DcMotorEx intakeMotor;

        public Shooter(HardwareMap hardwareMap) {
            shooterServo = hardwareMap.get(CRServo.class, "shooter_servo");
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // Configuração de frenagem para maior precisão
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
                    shooterMotor.setVelocity(2000);
                    intakeMotor.setPower(1);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double velocidadeAtual = Math.abs(shooterMotor.getVelocity());

                if (velocidadeAtual > 1960) {
                    shooterServo.setPower(-1);
                } else {
                    shooterServo.setPower(0);
                }

                // Corrigido de 18.0 para 3.0 segundos para não perder tempo no autônomo
                if (timer.seconds() > 10.0) {
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
                intakeMotor.setPower(1.0);
                return false;
            }
        }

        public class IntakeOffAction implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(0.0);
                return false;
            }
        }
    }

    public class AutoAimAction implements Action {
        private final TankDrive drive;
        private static final double DRIVE_GAIN_YAW = 0.03;
        private static final double MAX_AUTO_TURN = 0.5;
        private static final double ALIGNMENT_THRESHOLD_DEGREES = 1.0;

        private ElapsedTime timer;
        private boolean initialized = false;
        private final double timeLimitSeconds;

        public AutoAimAction(TankDrive drive, double timeLimitSeconds) {
            this.drive = drive;
            this.timeLimitSeconds = timeLimitSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                initialized = true;
            }

            // IMPORTANTE: Atualiza a odometria do RoadRunner mesmo controlando manualmente
            drive.updatePoseEstimate();

            if (timer.seconds() >= timeLimitSeconds) {
                setDrivePowers(0, 0);
                packet.put("AutoAim", "TIMEOUT - Disparando");
                return false;
            }

            AprilTagDetection detection = getAprilTagById(DESIRED_TAG_ID);

            if (detection == null) {
                packet.put("AutoAim", "Procurando Tag...");
                setDrivePowers(0, 0);
                return true;
            }

            double bearingError = detection.ftcPose.bearing;
            packet.put("AutoAim Erro", bearingError);

            if (Math.abs(bearingError) < ALIGNMENT_THRESHOLD_DEGREES) {
                setDrivePowers(0, 0);
                packet.put("AutoAim", "ALINHADO");
                return false;
            }

            // CORREÇÃO DE CÁLCULO: Removido o sinal negativo.
            // Tag à esquerda (Bearing +) -> TurnPower + -> Left Motor (-) -> Gira Esquerda (CCW)
            double turnPower = bearingError * DRIVE_GAIN_YAW;
            turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double leftPower = -turnPower;
            double rightPower = turnPower;

            // Feedforward estático para vencer inércia
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

            setDrivePowers(leftPower, rightPower);
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

        Action autoAim = new AutoAimAction(drive, 2.0);

        Action trajectoryAction = drive.actionBuilder(initialPose)
                .stopAndAdd(autoAim)
                .stopAndAdd(shooter.shoot())
                .lineToX(2)
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Aguardando start");
            AprilTagDetection det = getAprilTagById(DESIRED_TAG_ID);
            if (det != null) {
                telemetry.addData("Tag Detectada", "ID %d a %.2f graus", det.id, det.ftcPose.bearing);
            } else {
                telemetry.addData("Tag", "Procurando...");
            }
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(trajectoryAction);

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