import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WheelEncoderValueTester extends OpMode {
    int motorInitialPosition;

    double wheelRadius = 3.5;  //*** in inches***

    int encoderTick = 4096;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    double initialFrontLeft;
    double initialFrontRight;
    double initialBackLeft;
    double initialBackRight;

    double currentFrontLeft;
    double currentFrontRight;
    double currentBackLeft;
    double currentBackRight;

    boolean stop = false;

    double initialEncoderVal;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initialFrontLeft = frontLeftMotor.getCurrentPosition();
        initialFrontRight = frontRightMotor.getCurrentPosition();
        initialBackLeft = backLeftMotor.getCurrentPosition();
        initialBackRight = backRightMotor.getCurrentPosition();

        initialEncoderVal = (initialFrontLeft + initialFrontRight + initialBackLeft + initialBackRight) / 4;
    }

    @Override
    public void loop () {
        if (!stop) {
            frontLeftMotor.setPower(0.25);
            frontRightMotor.setPower(0.25);
            backLeftMotor.setPower(0.25);
            backRightMotor.setPower(0.25);
        }

        if (gamepad1.right_bumper) {
            stop = true;
        }

        currentFrontLeft = frontLeftMotor.getCurrentPosition();
        currentFrontRight = frontRightMotor.getCurrentPosition();
        currentBackLeft = backLeftMotor.getCurrentPosition();
        currentBackRight = backRightMotor.getCurrentPosition();

        double currentEncoderVal = (currentFrontLeft + currentFrontRight + currentBackLeft + currentBackRight) / 4;

        telemetry.addLine("Initial encoder val..." + initialEncoderVal);
        telemetry.addLine("Final encoder val..." + currentEncoderVal);
        telemetry.addLine("Encoder difference..." + (currentEncoderVal - initialEncoderVal));
    }

}