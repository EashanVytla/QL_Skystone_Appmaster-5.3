package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "SClamp Tuner")
//@Disabled
public class GrabberTuner extends OpMode {
    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    boolean previous = false;
    boolean previous2 = false;
    boolean previous3 = false;
    boolean previous4 = false;
    boolean previous5 = false;
    boolean previous6 = false;
    boolean previous7 = false;
    boolean previous8 = false;
    boolean clamp = false;

    Caching_Servo SClamp;
    Caching_Servo SArm;
    Grabber grabber;
    double i = 0.0;

    public void init() {
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        SClamp = new Caching_Servo(hardwareMap, "sclamp");
        SArm = new Caching_Servo(hardwareMap, "sarm");
        grabber = new Grabber(hardwareMap);
        SClamp.setPosition(0.1);
        SArm.setPosition(0.0);
    }

    public boolean isPressG(boolean value, boolean previous){
        return value && !previous;
    }

    public void loop() {
        if(gamepad1.dpad_up){
            if(i != 1){
                i += 0.001;
            }
            SArm.setPosition(i);
        }else if(gamepad1.dpad_down){
            if(i != 0){
                i -= 0.001;
            }
            SArm.setPosition(i);
        }

        if(gamepad1.dpad_right){
            if(i != 1){
                i += 0.001;
            }
            SClamp.setPosition(i);
        }else if(gamepad1.dpad_left){
            if(i != 0){
                i -= 0.001;
            }
            SClamp.setPosition(i);
        }

        if(isPressG(gamepad1.a, previous3)){
            if(clamp){
                SClamp.setPosition(0.325); //Unclamp
                //grabber.unclamp();
                clamp = false;
            }else{
                SClamp.setPosition(0.1); //Clamp
                //grabber.clamp();
                clamp = true;
            }
        }else if(isPressG(gamepad1.b, previous4)){
            SArm.setPosition(0.3); //Partial
            //grabber.partial();
        }else if(isPressG(gamepad1.x, previous5)){
            SArm.setPosition(0.0); //All the way up
            //grabber.goUp();
        }else if(isPressG(gamepad1.y, previous6)){
            SArm.setPosition(0.425); //Down Position
            //grabber.goDown();
        }else if(isPressG(gamepad1.right_bumper, previous7)){
            SArm.setPosition(0.157); //4rth Stone Position ALL DEPOSIT POSITIONS
            //grabber.deposit();
        }else if(isPressG(gamepad1.left_bumper, previous8)){
            SArm.setPosition(0.113); //4rth Stone Position ALL DEPOSIT POSITIONS
            //grabber.deposit();
        }

        telemetry.addData("Position SClamp", SClamp.getServo().getPosition());
        telemetry.addData("Position SArm", SArm.getServo().getPosition());

        previous = gamepad1.dpad_up;
        previous2 = gamepad1.dpad_down;
        previous3 = gamepad1.a;
        previous4 = gamepad1.b;
        previous5 = gamepad1.x;
        previous6 = gamepad1.y;
        previous7 = gamepad1.right_bumper;
        previous8 = gamepad1.left_bumper;

        SArm.write();
        SClamp.write();
        //grabber.write();
    }
}
