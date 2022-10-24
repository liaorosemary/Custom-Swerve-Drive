package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class Printer extends OpMode {
    File dir = AppUtil.ROBOT_DATA_DIR;
    File file = new File(dir, "data_log.txt");

    FileWriter myWriter;

    @Override
    public void init() {
        try{
            file.createNewFile();

            myWriter = new FileWriter(file);
            myWriter.write("there");
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

    }
}
