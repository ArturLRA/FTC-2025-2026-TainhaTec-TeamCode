package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//Primeiro eu declaro os "imports". Os dois primeiros são "import com.qualcomm.eventloop.opmode.LinearOpMode;" e "import com.qualcomm.eventloop.opmode.Autonomous; Nesse caso eu estou fazendo um autonomo, mas se fosse um teleoperado seria diferente essa segunda linha

@Disabled
@Autonomous(name="Autonomo: Treinamente2", group="Linear OpMode")
public class AutonomoTreinamente2 extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // No segundo eu declaro os valores dos motores no começo do código, que no caso é zero ou null usando o "private"
    private final double TICKS_PER_ROTATION = 292.0;
    private final double WHEEL_DIAMETER_INCHES = 3.54;
    private final double TRACK_WIDHT_INCHES = 14.0;

    // No terceiro eu declaro o tick do motor primeiro usando o "private final double TICKS_PER_ROTATION", depois eu declaro o diametro da roda usando o pivate final double de novo só que dessa vez eu coloco "WHEEL_DIANMETER_INCHES". E por ultimo eu declaro a circunferencia da roda usando "TRACK_WIDTH_ICHES". TUDO ISSO EM POLEGADAS
    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//Aqui nessa parte eu declaro os nomes dos motores

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//Já nessa parte eu declaro as direções em que a rodas vão girar

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

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        double turnCircumference = TRACK_WIDHT_INCHES * Math.PI;
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
