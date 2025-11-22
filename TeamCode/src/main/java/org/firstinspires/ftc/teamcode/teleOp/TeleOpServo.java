package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpServo", group="Linear OpMode")
public class TeleOpServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(0.0);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            if(gamepad1.a) {
                servo.setPosition(1.0);
            }else if(gamepad1.b) {
                servo.setPosition(0.5);
            }else {
                servo.setPosition(0.0);
            }
            telemetry.addData("Pos comandada", "%.3f", servo.getPosition());

            telemetry.update();
        }
    }

}