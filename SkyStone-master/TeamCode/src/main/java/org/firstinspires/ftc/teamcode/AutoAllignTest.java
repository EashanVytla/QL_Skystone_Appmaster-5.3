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
@Disabled
public class AutoAllignTest extends OpMode {
    double headingerror;
    double kpA = 1.0; //FINAL Regular turn: 0.39
    double kiA = 0.0; //FINAL Regular turn: 0.005
    double kdA = 0.0; //FINAL Regular turn: 0.2
    Mecanum_Drive drive;
    double angle;
    double prevheading = 0.0;

    double integralError = 0.0;
    long prev_time = System.currentTimeMillis();
    double dt = 0.0;
    double prevtime = 0.0;

    Flipper flip;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    Intake intake;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap, telemetry);

        flip = new Flipper(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        flip.grabPlatform();
        intake.close();
        intake.write();
    }

    public void loop(){
        flip.grabPlatform();

        dt = System.currentTimeMillis() - prev_time;
        prevtime = System.currentTimeMillis();
        angle = drive.angleWrap(drive.getExternalHeading());
        headingerror = Math.PI/2 - angle;

        if (Math.abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - Math.abs(headingerror));
            }
            else{
                headingerror = ((Math.PI * 2) - Math.abs(headingerror));
            }
        }

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

