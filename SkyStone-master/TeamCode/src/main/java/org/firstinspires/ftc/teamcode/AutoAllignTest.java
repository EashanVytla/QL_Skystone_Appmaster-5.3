package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name = "autoalligntest", group = "proto")
public class AutoAllignTest extends OpMode {
    double headingerror;
    double kpA = 0.39; //0.35625
    double kiA = 0.008; //0.005
    double kdA = 0.2; //0.2    0.3
    Mecanum_Drive drive;
    double angle;
    double prevheading = 0.0;

    double integralError = 0.0;
    long prev_time = System.currentTimeMillis();
    double dt = 0.0;
    double prevtime = 0.0;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap);
    }

    public void loop(){
        dt = System.currentTimeMillis() - prev_time;
        prevtime = System.currentTimeMillis();
        angle = drive.angleWrap(drive.getExternalHeading());
        headingerror = Math.PI/2 - angle;

        double prop = headingerror * kpA;
        double integral = integralError * kiA;
        double deriv = (headingerror - prevheading) * kdA / dt;

        double power = prop + integral + deriv;
        if (Math.abs(power) < 0.2){
            integralError += headingerror;
        }
        prevheading = headingerror;

        drive.setPower(0.0, 0.0, Range.clip(power, -1, 1));

        telemetry.addData("error", Math.toDegrees(headingerror));
        telemetry.addData("angle", Math.toDegrees(angle));

        drive.read(hub.getBulkInputData());
        drive.write();
    }

}

