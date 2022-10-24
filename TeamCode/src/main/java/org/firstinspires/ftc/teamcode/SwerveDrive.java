package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class SwerveDrive extends OpMode {

    // Controller inputs
    double controllerAngle;
    double x, y;

    // Wheel motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    // For printing to a log file
    String resultsLog = "";
    File dir = AppUtil.ROBOT_DATA_DIR;
    File file = new File(dir, "data_log.txt");
    FileWriter myWriter;

    // Constants
    double fullRotationTick = 4096 * 2;

    CRServo[] servos = new CRServo[4];
    DcMotor[] servoEncoders = new DcMotor[4];
    int [] directions = {1, 1, 1, 1};

    @Override
    public void init() {
        // Servos and Encoders
        int j = 0;
        for (String i : new String[]{"front_left", "front_right", "back_left", "back_right"}) {
            servos[j] = hardwareMap.get(CRServo.class, i);
            servoEncoders[j] = hardwareMap.get(DcMotor.class, i + "_encoder");
            j ++;
        }

        // Wheel Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        x = gamepad1.right_stick_x;
        y = -gamepad1.right_stick_y;    // invert, so that up is +, down is -
        controllerAngle = Math.atan2(y, x); // input angle

        double speed = 0.5 * Math.sqrt(x*x + y*y);  // ***Will change to use normalization

        controllerAngle = getAngleInFullRotation(controllerAngle);  // Convert controller angle to new range

        wheelToTargetAngle(controllerAngle, 0);
        wheelToTargetAngle(controllerAngle, 1);
        wheelToTargetAngle(controllerAngle, 2);
        wheelToTargetAngle(controllerAngle, 3);

        frontLeftMotor.setPower(speed * directions[0]);
        frontRightMotor.setPower(speed * directions[1]);
        backLeftMotor.setPower(speed * directions[2]);
        backRightMotor.setPower(speed * directions[3]);
    }

    public void wheelToTargetAngle(double targetAngle, int idx){
        double currentAngle = getCurrentAngle(idx);

        int direction = 1; // clockwise, (direction of wheel rotation)

        double alpha = targetAngle - currentAngle;
        double beta = 2 * Math.PI - Math.abs(alpha);

        double chosenAngleDiff = 0;

        // Since closest angle would allow wheel to turn no more than 90 degrees:

        if (Math.abs(alpha) <= (Math.PI / 2)) {
            chosenAngleDiff = Math.abs(alpha);
            if (alpha > 0) {
                direction = -1;
            }
        } else if (beta <= (Math.PI / 2)) {
            chosenAngleDiff = beta;
            if (alpha < 0) {
                direction = -1;
            }
        }
        else {      // Switch current angle to explementary angle for shortest rotational path
            directions[idx] *= -1;   // Switch direction of wheel turn, (and change current angle to match it, which will be reflected in getCurrentAngle(), when called again)
        }

        servos[idx].setPower(direction * (chosenAngleDiff)/(Math.PI / 2));

        telemetry.addLine("target: " + Math.toDegrees(targetAngle) + "; current: " + Math.toDegrees(currentAngle) + "; real current angle: " + servoEncoders[idx].getCurrentPosition());
    }

    double getCurrentAngle(int enc_idx) {
        // Using theta = s / r math (radian math), solve for corresponding angle on unit circle given encoder values (range from -PI to PI)

        // Get the modulus, since encoder tick rotation can pass a full turn
        double s = -servoEncoders[enc_idx].getCurrentPosition() % fullRotationTick;     // Since encoders report positively when moving clockwise, need to * -1
        s += s >= 0 ? 0 : fullRotationTick;

        double theta = (s / fullRotationTick) * (2 * Math.PI);

        if (directions[enc_idx] == -1) {     // Record angle as the explementary angle if the direction is inversed
            theta -= Math.PI;
            theta = getAngleInFullRotation(theta);
        }
        return theta;
    }

    double getAngleInFullRotation(double alpha) {
        // Angle between 0 and 2 * PI
        if (alpha < 0) {
            alpha += 2 * Math.PI;
        }
        return alpha;
    }

    @Override
    public void stop() {
        try{
            file.createNewFile();

            myWriter = new FileWriter(file);
            myWriter.write(resultsLog);
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}