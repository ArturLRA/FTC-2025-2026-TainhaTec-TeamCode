package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.Drawing;
import org.firstinspires.ftc.teamcode.roadRunner.TankDrive;



@Autonomous(name = "AutonomoSimples", group = "Autonomous")
public class autonomoTeste extends LinearOpMode {

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(30)
                .waitSeconds(1)
                .setTangent(Math.toRadians(90));

        Action trajectoryAction = tab1.build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Aguardando start");
            telemetry.addData("Posição detectada", visionOutputPosition);
            telemetry.update();

        }
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

        }


    }
}