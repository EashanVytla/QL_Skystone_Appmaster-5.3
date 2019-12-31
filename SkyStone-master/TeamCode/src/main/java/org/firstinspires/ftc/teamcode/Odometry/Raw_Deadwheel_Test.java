package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Odometry.MA3_Encoder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "Raw_Deadwheel_Test", group = "Odometry")
public class Raw_Deadwheel_Test extends OpMode {
    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    MA3_Encoder leftWheel;
    MA3_Encoder rightWheel;
    MA3_Encoder strafeWheel;

    public void init() {
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftWheel = new MA3_Encoder("a3", hardwareMap, 0.0);
        rightWheel = new MA3_Encoder("a4", hardwareMap, 0.0);
        strafeWheel = new MA3_Encoder("a1", hardwareMap, 0.0);
    }

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        telemetry.addData("Left Wheel: ", leftWheel.getRawDist(data));
        telemetry.addData("Right Wheel: ", rightWheel.getRawDist(data2));
        telemetry.addData("Strafe Wheel: ", strafeWheel.getRawDist(data));
    }
}
