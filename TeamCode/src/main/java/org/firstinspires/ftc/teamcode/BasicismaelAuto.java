package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Basic:ismaelAuto", group="Linear OpMode")

public class BasicismaelAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        double valor_encoder = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            valor_encoder = leftDrive.getCurrentPosition()/560;
            valor_encoder = rightDrive.getCurrentPosition()/560;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            leftDrive.setTargetPosition(560);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setPower(1000);

            rightDrive.setTargetPosition(560);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setPower(1000);
        }

    }
}
