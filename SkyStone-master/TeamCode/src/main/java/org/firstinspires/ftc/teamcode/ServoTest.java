package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest",group = "Proto")
public class ServoTest extends OpMode {
    Servo servo1;
    Servo servo2;
    Double i;

    public void init(){
        servo1 = hardwareMap.get(Servo.class, "grabber_1");
        servo2 = hardwareMap.get(Servo.class, "grabber_2");
        i = 0.0;
    }

    public void loop(){
        if(gamepad1.dpad_up){
            servo1.setPosition(i);
            i = i + 0.1;
        }else if(gamepad1.dpad_down){
            servo1.setPosition(i);
            i = i - .01;
        }else if(gamepad1.dpad_left){
            servo2.setPosition(i);
            i = i + .01;
        }else if(gamepad1.dpad_right){
            servo2.setPosition(i);
            i = i - .01;
        }else{
            servo2.setPosition(i);
        }
    }
}
