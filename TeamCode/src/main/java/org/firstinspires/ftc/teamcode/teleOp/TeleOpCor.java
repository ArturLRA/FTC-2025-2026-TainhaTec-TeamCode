package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpCor", group="Linear OpMode")
public class TeleOpCor extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx armDrive = null;

    private CRServo servoGarra1 = null;
    private CRServo servoGarra2 = null;

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private static double p = 0.05, i = 0.08, d = 0.0012; //Proporcional, Integral, Derivativo
    private static double f = 0.05; //feedFoward
    private PIDFController pidf;

    private int inicialArmPosition = 25, upArmPosition = 100, midleArmPosition = 60;

    private int armTargetPosition;
    private final int angleTolerance = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotorEx.class, "arm_drive");

        servoGarra1 = hardwareMap.get(CRServo.class, "servo_garra1");
        servoGarra2 = hardwareMap.get(CRServo.class, "servo_garra2");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        colorSensor.setGain(15);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.FORWARD);

        pidf = new PIDFController(p, i, d, f);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn =  gamepad1.right_stick_x;
            leftPower = Range.clip(drive - turn, -1.0, 1.0);
            rightPower = Range.clip(drive + turn, -1.0, 1.0);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            if (gamepad1.a) {
                servoGarra1.setPower(1.0);
                servoGarra2.setPower(1.0);
            } else {
                servoGarra1.setPower(0);
                servoGarra2.setPower(0);
            }

            if (gamepad1.x) {
                armTargetPosition = inicialArmPosition;
            } else if (gamepad1.b) {
                armTargetPosition = upArmPosition;
            } else if (gamepad1.y) {
                armTargetPosition = midleArmPosition;
            }

            updateArmPIDF();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            addSensorTelemetry();
            telemetry.update();
        }
    }

    public void updateArmPIDF() {
        int currentPosition = armDrive.getCurrentPosition();
        double pidfPower;

        int error = armTargetPosition - currentPosition;

        if (Math.abs(error) <= angleTolerance) {
            pidf.reset();
            pidfPower = f;
        } else {
            pidfPower = pidf.calculate(currentPosition, armTargetPosition);
        }

        armDrive.setPower(pidfPower);

        telemetry.addData("Arm Power (PIDF)", "%.3f", pidfPower);
    }

    public void addSensorTelemetry() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double distance = distanceSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Distancia (cm)", "%.2f", distance);

        telemetry.addData("Vermelho", "%.3f", colors.red);
        telemetry.addData("Verde", "%.3f", colors.green);
        telemetry.addData("Azul", "%.3f", colors.blue);
        telemetry.addData("Brilho (Alpha)", "%.3f", colors.alpha);
    }
}