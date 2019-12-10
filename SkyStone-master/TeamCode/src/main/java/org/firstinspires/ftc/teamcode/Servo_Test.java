package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Servo Test", group = "Diagnostics")
public class Servo_Test extends OpMode {
    Caching_Servo servo;

    long loop_count = 0;

    public void init(){
        servo = new Caching_Servo(hardwareMap, "Deposit");
    }

    public void loop(){
        servo.setPosition(1.0 - (loop_count / 250.0));
        servo.write();
        loop_count++;
    }
}
