package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name = "test_bulk_read", group = "REV")
@Disabled
public class test_bulk_read extends OpMode {
    RevBulkData data;
    DcMotor motor;
    ExpansionHubEx hub;
    long prev_time = System.currentTimeMillis();

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        motor = hardwareMap.get(DcMotor.class, "test_motor");
    }

    public void loop(){
        data = hub.getBulkInputData();
        motor.setPower(1.0);
        telemetry.addData("Pos: ", data.getMotorCurrentPosition(motor));
        telemetry.addData("Cycle Time: ", System.currentTimeMillis() - prev_time);
        prev_time = System.currentTimeMillis();
    }
}
