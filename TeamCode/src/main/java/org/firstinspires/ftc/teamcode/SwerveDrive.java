/**
 Some of these functions are directly copied, or copied then modified from the functions in the test methods
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class SwerveDrive extends OpMode {

    int wheelTurnDirection = 1;

    String resultsLog;

    double controllerAngle;

    double x, y;

    double rotationTickRadius = 1; // Inches of radius of rotation for servos (obtained from WheelRotationTests)

    double fullRotationTick = 4096 * 2;

    // ticks for half turn / pi, values obtained from WheelRotationTests
    double tickAngleRatio = 4096 / Math.PI;

    // Controls wheel angles
    CRServo frontLeftServo;
    CRServo frontRightServo;
    CRServo backLeftServo;
    CRServo backRightServo;

    // False motors to get servo encoder values
    DcMotor frontLeftServoEncoder;
    DcMotor frontRightServoEncoder;
    DcMotor backLeftServoEncoder;
    DcMotor backRightServoEncoder;

    // Wheel motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;


    File dir = AppUtil.ROBOT_DATA_DIR;
    File file = new File(dir, "data_log.txt");

    FileWriter myWriter;

    @Override
    public void init() {
        // Servos
        frontLeftServo = hardwareMap.get(CRServo.class, "front_left");
        frontRightServo = hardwareMap.get(CRServo.class, "front_right");
        backLeftServo = hardwareMap.get(CRServo.class, "back_left");
        backRightServo = hardwareMap.get(CRServo.class, "back_right");

        // Servo Encoders
        frontLeftServoEncoder = hardwareMap.get(DcMotor.class, "front_left_encoder");
        frontRightServoEncoder = hardwareMap.get(DcMotor.class, "front_right_encoder");
        backLeftServoEncoder = hardwareMap.get(DcMotor.class, "back_left_encoder");
        backRightServoEncoder = hardwareMap.get(DcMotor.class, "back_right_encoder");

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

        double speed = 0.5 * Math.sqrt(x*x + y*y);

        /* Need to modify before using !!!***************************************************************
        // ***NOTE: closest angle not necessarily returned in angle range!!!
        wheelToAngle(getAngleWithinRange(getClosestTargetAngle(getCurrentAngle(frontLeftServoEncoder))), frontLeftServo, frontLeftServoEncoder);
        wheelToAngle(getAngleWithinRange(getClosestTargetAngle(getCurrentAngle(frontRightServoEncoder))), frontRightServo, frontRightServoEncoder);
        wheelToAngle(getAngleWithinRange(getClosestTargetAngle(getCurrentAngle(backLeftServoEncoder))), backLeftServo, backLeftServoEncoder);
        wheelToAngle(getAngleWithinRange(getClosestTargetAngle(getCurrentAngle(backRightServoEncoder))), backRightServo, backRightServoEncoder);

         */
        resultsLog += "\n";

        controllerAngle = getAngleWithinNewRange(controllerAngle);

        newWheelToAngle(controllerAngle, getAngleWithinNewRange(getCurrentAngle(frontLeftServoEncoder)), frontLeftServo);
        newWheelToAngle(controllerAngle, getAngleWithinNewRange(getCurrentAngle(frontRightServoEncoder)), frontRightServo);
        newWheelToAngle(controllerAngle, getAngleWithinNewRange(getCurrentAngle(backLeftServoEncoder)), backLeftServo);
        newWheelToAngle(controllerAngle, getAngleWithinNewRange(getCurrentAngle(backRightServoEncoder)), backRightServo);


        frontLeftMotor.setPower(speed * wheelTurnDirection);
        frontRightMotor.setPower(speed * wheelTurnDirection);
        backLeftMotor.setPower(speed * wheelTurnDirection);
        backRightMotor.setPower(speed * wheelTurnDirection);
    }

    public void newWheelToAngle(double targetAngle, double currentAngle, CRServo servo){

        int direction = 1; // clockwise

//        if (currentAngle < (targetAngle - Math.toRadians(10)) || currentAngle > (targetAngle + Math.toRadians(10))) {
        double alpha = targetAngle - currentAngle;
        double beta = 2 * Math.PI - Math.abs(alpha);

        double chosenAngleDiff = 0;


        if (Math.abs(alpha) <= (Math.PI / 2)) {
            chosenAngleDiff = Math.abs(alpha);
            if (alpha > 0) {
                direction *= -1;
            }
        } else if (beta <= (Math.PI / 2)) {
            chosenAngleDiff = beta;
            if (alpha < 0) {
                direction *= -1;
            }
        }
        else {
//            direction *= -1;
            wheelTurnDirection *= -1;
            telemetry.addLine("use other angle: " + Math.toDegrees(getAngleWithinNewRange(targetAngle - Math.PI)));
            newWheelToAngle(getAngleWithinNewRange(targetAngle - Math.PI), currentAngle, servo);
            return;
        }

        servo.setPower(direction * 0.5 * (chosenAngleDiff)/(Math.PI / 2));

        telemetry.addLine("setting power to: " + (direction * 0.5));
//        }

        telemetry.addLine("current: " + Math.toDegrees(currentAngle) + "; target: " + Math.toDegrees(targetAngle));

    }

    public void wheelToAngle(double targetAngle, CRServo wheelServo, DcMotor currentEncoder) {

        double currentAngle = getCurrentAngle(currentEncoder);  // Already in range
        telemetry.addLine("Current Angle: " + Math.toDegrees(currentAngle) + ";Target Angle: " + Math.toDegrees(targetAngle));

//        double targetEncoderPosition = -(targetAngle * tickAngleRatio);

//        resultsLog += ("\nx: " + x + "; y: " + y + "; Target angle: " + targetAngle + "; Target Encoder Position: " + targetEncoderPosition + "; Current encoder position: " + currentEncoderPosition + "; tickAngleRatio: " + tickAngleRatio);

        /*
        if (currentEncoderPosition > targetEncoderPosition + 100) {
            telemetry.addLine("power to: " + (1 * (currentEncoderPosition - targetEncoderPosition) / 4096));
            wheelServo.setPower(Math.min(-0.5, -1 * (currentEncoderPosition - targetEncoderPosition) / 4096));
        }
        else if (currentEncoderPosition < targetEncoderPosition - 100) {
            telemetry.addLine("power to: " + (1 * (targetEncoderPosition - currentEncoderPosition) / 4096));
            wheelServo.setPower(Math.min(0.5, 1 *  (targetEncoderPosition - currentEncoderPosition) / 4096));
        }
         */


        double inRangeTargetAngle = getAngleWithinRange(targetAngle);

        double power = 0;

        // Two cases and consequently two ways to meet target angle
        if (currentAngle * inRangeTargetAngle >= 0) { // if target and current angle both have same sign
            if (inRangeTargetAngle < currentAngle) {
                power = -0.25;
            }
            else if (inRangeTargetAngle > currentAngle) {
                power = 0.25;
            }
        }
        else { // if target and current angle don't have same sign
            int quadrant = getAngleQuadrant(currentAngle);

            switch (quadrant) {
                case 1:
                    if (currentAngle > inRangeTargetAngle) {
                        power = -0.25;
                    }
                    break;
                case 2:
                    if (currentAngle < targetAngle) {
                        power = 0.25;
                    }
                    break;
                case 3:
                    if (currentAngle < targetAngle) {
                        power = -0.25;
                    }
                    break;
                case 4:
                    if (currentAngle < inRangeTargetAngle)
                        power = 0.25;
                    break;
            }
        }

//        wheelServo.setPower(-1 * power);

    }

    double getCurrentAngle(DcMotor currentEncoder) {
        // Using theta = s / r math (radian math), solve for corresponding angle on unit circle given encoder values (range from -PI to PI)
        double s = -currentEncoder.getCurrentPosition() % fullRotationTick;    // Get the modulus, since encoder tick rotation can pass a full turn
        s += s >= 0 ? 0 : fullRotationTick;

//        double theta = s / (rotationTickRadius * tickAngleRatio);

        double theta = (s / fullRotationTick) * 2 * Math.PI;

        if (wheelTurnDirection == -1) {
            theta -= 2 * Math.PI;
            theta = getAngleWithinNewRange(theta);
        }

//        theta = getAngleWithinRange(theta);

        telemetry.addLine("theta: " + Math.toDegrees(theta));
        return theta;
    }

    double getClosestTargetAngle(double currentAngle){
        // ***NOTE: closest angle not necessarily returned in angle range!!!
        double controllerAngle = Math.atan2(y, x);
        currentAngle = getAngleWithinRange(currentAngle);

        // The limits could sometimes surpass range that atan2 would return, so would need to convert the limits
        double limit1 = getAngleWithinRange(currentAngle - (Math.PI / 2));
        double limit2 = getAngleWithinRange(currentAngle + (Math.PI / 2));

        /**
         * CAN get rid of the repetitive parts in 4 cases!!!
         */
        double targetAngle = 0;

        /**
         * CAN MAYBE PUT QUADRANT 1 + 3 TOGETHER AND 2 + 4 TOGETHER!!!
         */

        // 4 possibilities for where current angle lies (per quadrant)
        if (currentAngle >= 0 && currentAngle <= (Math.PI / 2)) { // Quadrant 1

            // If it's within the 90 degree range
            if (controllerAngle >= limit1 && controllerAngle <= limit2) {
                targetAngle = controllerAngle;
            }
            // Otherwise, the other possible value is within the 90 degree range
            else {
                targetAngle = controllerAngle + Math.PI;
            }
        }
        else if (currentAngle > (Math.PI / 2) && currentAngle <= (Math.PI)) { // Quadrant 2
            if (controllerAngle >= limit1 || controllerAngle <= limit2) {
                targetAngle = controllerAngle;
            }
            else {
                targetAngle = controllerAngle + Math.PI;
            }
        }
        else if (currentAngle <= -(Math.PI / 2)) { // Quadrant 3
            if (controllerAngle >= limit1 && controllerAngle <= limit2) {
                targetAngle = controllerAngle;
            }
            else {
                targetAngle = controllerAngle + Math.PI;
            }
        }
        else if (currentAngle > -(Math.PI / 2) && currentAngle < 0) { // Quadrant 4

            // If it's within the 90 degree range
            if (controllerAngle <= limit2 && controllerAngle >= limit1) {
                targetAngle = controllerAngle;
            }
            // Otherwise, the other possible value is within the 90 degree range
            else {
                targetAngle = controllerAngle + Math.PI;
            }
        }
        ///****targetAngle is not within range!!!
        return targetAngle;
    }

    double getAngleWithinNewRange(double alpha) {
        if (alpha < 0) {
            alpha += 2 * Math.PI;
        }
        return alpha;
    }

    double getAngleWithinRange(double alpha) {
        // Keeps the angle within the range of -PI to PI

        if (alpha > Math.PI) {
            alpha = -(Math.PI * 2 - alpha);
            // flip speed direction
        }
        else if (alpha < -Math.PI) {
            alpha = Math.PI + alpha;
            // flip speed direction
        }
        return alpha;
    }

    int getAngleQuadrant(double angle){
        // NOTE: angle must be in -PI to PI range

        int quadrant = 0;

        // 4 Cases (quadrants)
        if (angle >= 0 && angle <= (Math.PI / 2)) { // Quadrant 1
            return 1;
        }
        else if (angle > (Math.PI / 2) && angle <= (Math.PI)) { // Quadrant 2
            return 2;
        }
        else if (angle <= -(Math.PI / 2)) { // Quadrant 3
            return 3;
        }
        else { // Quadrant 4
            return 4;
        }
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