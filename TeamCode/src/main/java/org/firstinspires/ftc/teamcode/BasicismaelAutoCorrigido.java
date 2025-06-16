package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Basic:ismaelAuto", group="Linear OpMode")
public class BasicismaelAutoCorrigido extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final double TICKS_PER_ROTATION = 292.0;
    private final double WHEEL_DIAMETER_INCHES = 3.54;
    private final double TRACK_WIDTH_INCHES = 14.0;

    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        runInches(44, 0.4);
        turnDegrees(90, 0.4);
        runInches(22, 0.4);

        telemetry.addData("Status", "Complete");
        telemetry.update();
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
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);
    }

    public void turnDegrees(double degrees, double power) {
        double turnCircumference = TRACK_WIDTH_INCHES * Math.PI;

        double turnDistance = (turnCircumference * degrees) / 360.0;

        int turnTicks = convertInchesToTicks(turnDistance);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(turnTicks);
        rightDrive.setTargetPosition(-turnTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);
    }

    public int convertInchesToTicks(double inches) {
        return (int) ((TICKS_PER_ROTATION * inches) / (WHEEL_DIAMETER_INCHES * Math.PI));
    }
}

