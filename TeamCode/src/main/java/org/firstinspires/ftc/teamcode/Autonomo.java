package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Autonomo", group="autonomous")
public class Autonomo extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private CRServo servo = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 336;
    static final double WHEEL_DIAMETER_CM = 9.0;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        servo = hardwareMap.get(CRServo.class, "servo_claw");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED, 122, 122, 5.0);

        servo.setPower(0.6);

        encoderDrive(TURN_SPEED, 30, -30, 4.0);
        encoderDrive(DRIVE_SPEED, -61, -61, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void encoderDrive(double speed,
                             double leftCM, double rightCM,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftCM * COUNTS_PER_CM);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightCM * COUNTS_PER_CM);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}