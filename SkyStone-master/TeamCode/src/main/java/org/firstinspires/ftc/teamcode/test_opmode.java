package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test_rev", group = "REV")
public class test_opmode extends OpMode {
    DcMotor motor;

    public void init(){
        motor = hardwareMap.get(DcMotor.class, "test_motor");
    }

    public void loop(){
        motor.setPower(1.0);
    }
}
