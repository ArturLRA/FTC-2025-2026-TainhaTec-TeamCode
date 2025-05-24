package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Roda", group="Linear OpMode")

public class TeleOpRoda extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private CRServo servo = null;

    private static final double SERVO_POWER = 0.5;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        servo = hardwareMap.get(CRServo.class, "servo_claw");


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //Programação do motor
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double leftPower;
            double rightPower;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //Programação do servo

            if (gamepad1.a) {
                servo.setPower(SERVO_POWER);
            } else if (gamepad1.b) {
                servo.setPower(-SERVO_POWER);
            } else {
                servo.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }
}