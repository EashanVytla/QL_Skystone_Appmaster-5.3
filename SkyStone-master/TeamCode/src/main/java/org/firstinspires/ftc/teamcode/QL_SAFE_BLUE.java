package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "QL_Auto_Safe_BLUE")
@Disabled
public class QL_SAFE_BLUE extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    Mecanum_Drive drive;

    Flipper flip;
    Intake intake;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftWheel = new Dead_Wheel(new MA3_Encoder("a3", hardwareMap, 0.495));
        rightWheel = new Dead_Wheel(new MA3_Encoder("a4", hardwareMap, 1.365));
        strafeWheel = new Dead_Wheel(new MA3_Encoder("a1", hardwareMap, 2.464));
        drive = new Mecanum_Drive(hardwareMap);
        flip = new Flipper(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        rightWheel.getEncoder().reverse();
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();
        leftWheel.getEncoder().calibrate(data);
        rightWheel.getEncoder().calibrate(data2);
        strafeWheel.getEncoder().calibrate(data);
        leftWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5144 0.0361262
        rightWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642 * 2 * 0.797, 0.0); //1.50608 -0.221642
    }

    @Override
    public void start(){
        intake.kickout();
    }

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        flip.read();


        strafeWheel.update(data);

        if(strafeWheel.getDistance() >= 5){
            drive.setPower(0.0,0.0,0.0);
        }else{
            drive.setPower(0.0, -0.3, 0.0);
        }

    }
}
