package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "IMU_Tester", group = "Competition")
public class IMU_Tester extends OpMode {
    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    Mecanum_Drive drive;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap, telemetry);
    }

    public void loop(){
        RevBulkData data = hub.getBulkInputData();

        drive.read(data);

        telemetry.addData("IMU Angle: ", drive.angleWrap(drive.getExternalHeading()));
    }
}
