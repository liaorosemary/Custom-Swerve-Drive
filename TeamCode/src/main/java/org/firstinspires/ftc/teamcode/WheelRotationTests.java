/**
 Opmode description:
 Returns original, final and encoder difference values.

 Used to determine tick to angle ratio:
    - turn all wheels in a certain direction with roughly same angle change
        - obtain turn direction (positive or negative in clockwise or counter clockwise direction)
        - obtain tick difference

 Results:
    - All wheels:
        - positive turn direction: CLOCKWISE
        - tick difference for 1/2 turn (180 degrees) in clockwise direction: 4096

 *NOTE*: encoders always begin at 0, no matter the angle position...

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WheelRotationTests extends OpMode {

    double fullRotationTick = 4096 * 2;
    double rotationTickRadius = 1 * (4096 / Math.PI); // Convert radius of turn from 1 inch to ticks (obtained using returnAngleValues test)

    double x, y;

    // For running specific functions in loop()
    String nameOfTest = "closestTargetAngle";

    DcMotor frontLeftServoEncoder;
    DcMotor frontRightServoEncoder;
    DcMotor backLeftServoEncoder;
    DcMotor backRightServoEncoder;

    // Original (once program starts) tick values:
    double initialFrontLeft;
    double initialFrontRight;
    double initialBackLeft;
    double initialBackRight;

    // Current tick values:
    double currentFrontLeft;
    double currentFrontRight;
    double currentBackLeft;
    double currentBackRight;

    @Override
    public void init() {
        // Servo Encoders
        frontLeftServoEncoder = hardwareMap.get(DcMotor.class, "front_left_encoder");
        frontRightServoEncoder = hardwareMap.get(DcMotor.class, "front_right_encoder");
        backLeftServoEncoder = hardwareMap.get(DcMotor.class, "back_left_encoder");
        backRightServoEncoder = hardwareMap.get(DcMotor.class, "back_right_encoder");

        frontLeftServoEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightServoEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftServoEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightServoEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initialFrontLeft = frontLeftServoEncoder.getCurrentPosition();
        initialFrontRight = frontRightServoEncoder.getCurrentPosition();
        initialBackLeft = backLeftServoEncoder.getCurrentPosition();
        initialBackRight = backRightServoEncoder.getCurrentPosition();

    }

    @Override
    public void loop () {

        y = -gamepad1.right_stick_y;
        x = gamepad1.right_stick_x;

        currentFrontLeft = frontLeftServoEncoder.getCurrentPosition();
        currentFrontRight = frontRightServoEncoder.getCurrentPosition();
        currentBackLeft = backLeftServoEncoder.getCurrentPosition();
        currentBackRight = backRightServoEncoder.getCurrentPosition();

        switch (nameOfTest) {
            case "tickVal":
                // Return the tick values for each servo
                printTickValues("Front Left", initialFrontLeft, currentFrontLeft);
                printTickValues("Front Right", initialFrontRight, currentFrontRight);
                printTickValues("Back Left", initialBackLeft, currentBackLeft);
                printTickValues("Back Right", initialBackRight, currentBackRight);
                break;
            case "angleVal":
                // Input -encoder values, to match unit circle
                printAngleValues("Front Left", -currentFrontLeft);
                printAngleValues("Front Right", -currentFrontRight);
                printAngleValues("Back Left", -currentBackLeft);
                printAngleValues("Back Right", -currentBackRight);
                break;
            case "closestTargetAngle":
                // Get the closest angle to current angle
                printClosestTargetAngle("Front Left", -currentFrontLeft);
                printClosestTargetAngle("Front Right", -currentFrontRight);
                printClosestTargetAngle("Back Left", -currentBackLeft);
                printClosestTargetAngle("Back Right", -currentBackRight);
                break;
        }
    }

    void printTickValues(String wheelPosition, double initialEncoderVal, double currentEncoderVal) {
        // Find encoder initial, final and difference values
        telemetry.addLine(wheelPosition + " difference: " + (currentEncoderVal - initialEncoderVal) + "; Initial: " + initialEncoderVal + "; New: " + currentEncoderVal);
    }

    void printAngleValues(String wheelPosition, double currentEncoderVal) {

        double theta = getCurrentAngle(currentEncoderVal);

        theta = getAngleWithinRange(theta);    // Keep within range

        double thetaDegrees = theta / Math.PI * 180;

        telemetry.addLine(wheelPosition + " theta radians: " + theta + "; In degrees: " + thetaDegrees);
    }

    void printClosestTargetAngle(String wheelPosition, double currentEncoderVal){
        // ***NOTE: closest angle not necessarily returned in angle range!!!
        double controllerAngle = Math.atan2(y, x);

        double currentAngle = getCurrentAngle(currentEncoderVal);
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

        // Print values in degrees
        telemetry.addLine(wheelPosition + " current angle: " + Math.toDegrees(currentAngle) + "; Controller Angle: " + Math.toDegrees(controllerAngle) + "; Closest angle: " + Math.toDegrees(getAngleWithinRange(targetAngle)));
    }

    double getCurrentAngle(double currentEncoderVal) {
        // Using theta = s / r math (radian math)
        double s = currentEncoderVal % fullRotationTick;    // Get the modulus, since encoder tick rotation can pass a full turn
        double theta = s / rotationTickRadius;

        return theta;
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
}