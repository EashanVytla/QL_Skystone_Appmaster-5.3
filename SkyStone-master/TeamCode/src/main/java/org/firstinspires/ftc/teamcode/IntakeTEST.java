package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTEST", group = "Competition")
@Disabled
public class IntakeTEST extends OpMode {
    DcMotor intake1;
    DcMotor intake2;

    public void loop() {
        intake1.setPower(1);
        intake2.setPower(1);
    }

    public void init(){
        intake1 = hardwareMap.get(DcMotor.class, "intake_left");
        intake2 = hardwareMap.get(DcMotor.class, "intake_right");
    }
}
