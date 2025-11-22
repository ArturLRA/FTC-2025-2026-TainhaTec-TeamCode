package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="TeleOpRoboCameraMod", group="Linear OpMode")
public class TeleOpCameraMod extends LinearOpMode {

    // --- Variáveis do Drive e Mecanismos ---
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor shooterMotor = null;
    private CRServo shooterServo = null;

    // --- Variáveis da Câmera ---
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- Constantes de Controle (Ajuste Fino) ---

    // Resolução da câmera (usada para achar o centro)
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_CENTER_X = CAMERA_WIDTH / 2;

    // Ganho para girar o robô baseado no erro de pixels (X).
    // Ex: Erro de 100 pixels * 0.002 = 0.2 de força.
    private static final double DRIVE_GAIN_PIXEL = 0.002;

    // Potência Mínima para vencer o atrito (Feedforward)
    // Se o robô fizer barulho mas não girar, aumente isso (ex: 0.15)
    private static final double MIN_TURN_POWER = 0.08;

    // Constantes do Shooter
    private static final double SHOOTER_BASE_POWER = 0.4;
    private static final double SHOOTER_GAIN_PITCH = 0.02;

    private static final double MAX_AUTO_TURN = 0.4;   // Segurança
    private static final int DESIRED_EXPOSURE_MS = 6;
    private static final int DESIRED_GAIN = 250;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Iniciando Hardware...");
        telemetry.update();

        // 1. Hardware Map
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");
        shooterMotor    = hardwareMap.get(DcMotor.class, "shooter_motor");
        shooterServo    = hardwareMap.get(CRServo.class, "shooter_servo");

        configureDriveMotors();

        // 2. Câmera
        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(DESIRED_EXPOSURE_MS, DESIRED_GAIN);
        }

        telemetry.addData("Status", "Pronto. Pressione Play.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // --- Lógica Principal ---
            if (gamepad1.left_bumper) {
                // Modo Automático: Alinha usando o centro da imagem (pixels)
                alignRobotWithTagPixel();
            } else {
                // Modo Manual
                driveControlManual();
                shooterControlManual();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * NOVA FUNÇÃO: Alinha o robô tentando colocar a Tag no centro X da imagem.
     */
    private void alignRobotWithTagPixel() {
        AprilTagDetection detection = getPrimaryAprilTagDetection();

        if (detection != null) {
            // Pega a posição X do centro da tag na imagem (0 a 640)
            double tagX = detection.center.x;

            // Calcula o erro: Distância do centro da tag até o centro da tela
            // Se tagX < 320 (está na esquerda), erro é positivo.
            // Se tagX > 320 (está na direita), erro é negativo.
            double errorX = CAMERA_CENTER_X - tagX;

            // Calcula a força de giro (Proporcional)
            double turnPower = errorX * DRIVE_GAIN_PIXEL;

            // Adiciona uma força mínima (Feedforward) para vencer o atrito se o erro for pequeno
            if (Math.abs(errorX) > 10) { // Zona morta de 10 pixels
                if (turnPower > 0) {
                    turnPower += MIN_TURN_POWER;
                } else {
                    turnPower -= MIN_TURN_POWER;
                }
            }

            // Limita a velocidade
            turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            // Aplica nas rodas
            // Para girar para a esquerda (erro positivo), motores esquerdos vão para trás, direitos para frente.
            // Nota: Se girar para o lado errado, inverta os sinais aqui!
            double leftPower  = -turnPower;
            double rightPower = turnPower;

            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

            // --- Controle do Shooter (Mantido pela distância/pitch) ---
            double pitchError = detection.ftcPose.pitch;
            double autoShooterPower = SHOOTER_BASE_POWER + (Math.abs(pitchError) * SHOOTER_GAIN_PITCH);
            autoShooterPower = Range.clip(autoShooterPower, 0.0, 1.0);
            shooterMotor.setPower(autoShooterPower);

            // Telemetria para Debug
            telemetry.addLine("--- MIRA PIXEL ---");
            telemetry.addData("Tag X", "%.1f (Centro: %d)", tagX, CAMERA_CENTER_X);
            telemetry.addData("Erro X", "%.1f pixels", errorX);
            telemetry.addData("Turn Power", "%.3f", turnPower);

        } else {
            stopAllMotors();
            telemetry.addLine("--- MIRA PIXEL ---");
            telemetry.addLine("Procurando Tag...");
        }
    }

    private void stopAllMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        shooterMotor.setPower(0);
        shooterServo.setPower(0);
    }

    // --- Métodos Manuais e Configurações ---

    private void driveControlManual() {
        double drive = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;
        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }

    private void shooterControlManual() {
        if (gamepad1.right_bumper) {
            shooterMotor.setPower(0.75);
            shooterServo.setPower(-1.0);
        } else {
            shooterMotor.setPower(0.0);
            shooterServo.setPower(0.0);
        }
    }

    private void configureDriveMotors() {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
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
            builder.setCameraResolution(new android.util.Size(CAMERA_WIDTH, 480)); // Usa a constante de largura
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
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
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