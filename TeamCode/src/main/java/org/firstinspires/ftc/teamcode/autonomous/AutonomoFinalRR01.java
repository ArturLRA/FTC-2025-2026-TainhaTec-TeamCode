package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.Drawing;
import org.firstinspires.ftc.teamcode.roadRunner.TankDrive;

@Config
@Autonomous(name = "AutonomoFinalRR01", group = "Autonomous")
public class AutonomoFinalRR01 extends LinearOpMode {

    public class Shooter {
        private CRServo shooterServo;
        private DcMotorEx shooterMotor;

        public Shooter(HardwareMap hardwareMap) {
            shooterServo = hardwareMap.get(CRServo.class, "shooter_servo");
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");

            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public Action shoot() {
            return new ShooterOnAction();
        }

        public class ShooterOnAction implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooterMotor.setVelocity(2100);
                    timer = new ElapsedTime();
                    initialized = true;
                }

                double velocidadeAtual = Math.abs(shooterMotor.getVelocity());

                if (velocidadeAtual > 2000) {
                    shooterServo.setPower(-1);
                } else {
                    shooterServo.setPower(0);
                }

                if (timer.seconds() > 3.0) {
                    shooterMotor.setVelocity(0);
                    shooterServo.setPower(0);
                    return false;
                }
                return true;
            }
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        Shooter shooter = new Shooter(hardwareMap);

        int visionOutputPosition = 1;

        Action trajectoryAction = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(180))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Aguardando start");
            telemetry.addData("Posição detectada", visionOutputPosition);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Executando trajetória");
        telemetry.update();

        Actions.runBlocking(trajectoryAction);

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            Canvas fieldOverlay = packet.fieldOverlay();

            fieldOverlay.setStroke("#3F51B5");
            Drawing.drawRobot(fieldOverlay, pose);

            packet.put("x", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            telemetry.addData("Pose", pose);
            telemetry.update();

            com.acmerobotics.dashboard.FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}