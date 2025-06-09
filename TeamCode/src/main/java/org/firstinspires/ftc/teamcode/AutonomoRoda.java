package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "AutonomoRoda", group = "autonomous")
public class AutonomoRoda extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private CRServo servo = null;

    private IMU imu;
    private IMU.Parameters imuParams;

    private ElapsedTime runtime = new ElapsedTime();

    private final double TICKS_PER_ROTATION = 292.0;
    private final double WHEEL_DIAMETER_INCHES = 3.54;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        servo = hardwareMap.get(CRServo.class, "servo_claw");

        imu = hardwareMap.get(IMU.class, "imu");
        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(imuParams);
        imu.resetYaw();
        sleep(300);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        runInches(44, 0.4);

        turn(90, 0.5);

        runInches(22, 0.4);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    public void runInches(double inches, double power) {
        int ticks = convertInchesToTicks(inches);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(ticks);
        rightDrive.setTargetPosition(ticks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            showTelemetry();
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);
    }

    public void turn(double targetAngle, double maxPower) {

        double kP = 0.01;
        double kI = 0.0;
        double kD = 0.0015;

        double integral = 0;
        double previousError = 0;
        double previousTime = runtime.seconds();

        double minPower = 0.07;
        double tolerance = 1.5;
        int maxAttempts = 2;
        int attempt = 0;

        while (opModeIsActive() && attempt < maxAttempts) {
            boolean reachedTarget = false;

            imu.resetYaw();
            sleep(250);

            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive()) {
                double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double error = angleWrap(targetAngle - currentYaw);

                if (Math.abs(error) <= tolerance) {
                    reachedTarget = true;
                    break;
                }

                double currentTime = runtime.seconds();
                double deltaTime = currentTime - previousTime;
                if (deltaTime == 0) deltaTime = 0.001;

                integral += error * deltaTime;
                double derivative = (error - previousError) / deltaTime;

                double output = kP * error + kI * integral + kD * derivative;

                if (Math.abs(output) < minPower) {
                    output = minPower * Math.signum(output);
                }

                output = Math.max(-maxPower, Math.min(maxPower, output));

                leftDrive.setPower(output);
                rightDrive.setPower(-output);

                previousError = error;
                previousTime = currentTime;

                telemetry.addData("Yaw", currentYaw);
                telemetry.addData("Erro", error);
                telemetry.addData("Potência", output);
                telemetry.addData("Tentativa", attempt + 1);
                telemetry.update();

                idle();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(200);

            if (reachedTarget) break;

            attempt++;
        }
    }

    public double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public int convertInchesToTicks(double inches) {
        return (int) ((TICKS_PER_ROTATION * inches) / (WHEEL_DIAMETER_INCHES * Math.PI));
    }

    public void showTelemetry() {
        telemetry.addData("Run Time", runtime.toString());
        telemetry.addData("Encoder Left", leftDrive.getCurrentPosition());
        telemetry.addData("Encoder Right", rightDrive.getCurrentPosition());
        telemetry.update();
    }
}