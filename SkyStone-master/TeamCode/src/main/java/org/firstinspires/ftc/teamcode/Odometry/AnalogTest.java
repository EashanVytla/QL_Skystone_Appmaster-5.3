package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="Analog Dumper", group = "MA3")
@Disabled
public class AnalogTest extends OpMode {
    AnalogInput a0;
    AnalogInput a1;
    AnalogInput a2;
    AnalogInput a3;

    public void init(){
        a0 = hardwareMap.get(AnalogInput.class, "a0");
        a1 = hardwareMap.get(AnalogInput.class, "a1");
        a2 = hardwareMap.get(AnalogInput.class, "a2");
        a3 = hardwareMap.get(AnalogInput.class, "a3");
    }

    public void loop(){
        telemetry.addData("A0: ", a0.getVoltage());
        telemetry.addData("A1: ", a1.getVoltage());
        telemetry.addData("A2: ", a2.getVoltage());
        telemetry.addData("A3: ", a3.getVoltage());
    }
}
