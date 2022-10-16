import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;

@TeleOp
public class LimitTester extends OpMode {

    Map<String, Servo> servoDictionary1 = new HashMap<String, Servo>();

    String[] servoNames = {"front_left", "front_right", "back_left", "back_right"};

    String currentMotor = "front_left";
    double currentPosition = 0;

    boolean incrementPressed = false;

    @Override
    public void init() {
        // Servos
        for (String i : servoNames) {
            servoDictionary1.put(i, hardwareMap.get(Servo.class, i));
        }
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            currentPosition = 0;
            currentMotor = "front_left";
        }
        else if (gamepad1.b) {
            currentPosition = 0;
            currentMotor = "front_right";
        }
        else if (gamepad1.x) {
            currentPosition = 0;
            currentMotor = "back_left";
        }
        else if (gamepad1.y) {
            currentPosition = 0;
            currentMotor = "back_right";
        }

        if (gamepad1.right_bumper && !incrementPressed) {
            currentPosition += 0.05;
            incrementPressed = true;
        }
        else if (!gamepad1.right_bumper) {
            incrementPressed = false;
        }

        servoDictionary1.get(currentMotor).setPosition(currentPosition);

        telemetry.addLine(currentMotor + currentPosition);
    }
}