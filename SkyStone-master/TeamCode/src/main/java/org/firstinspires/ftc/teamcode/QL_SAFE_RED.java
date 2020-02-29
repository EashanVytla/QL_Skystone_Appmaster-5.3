package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "QL_Auto_PARK", group = "Competition")
//@Disabled
public class QL_SAFE_RED extends OpMode {
    /*
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    Mecanum_Drive drive;

    Flipper flip;


    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    */
    ElapsedTime mStateTime = new ElapsedTime();
    Intake intake;
    FlipperV2 flip;

    public void init(){
        intake = new Intake(hardwareMap);
        flip = new FlipperV2(hardwareMap, telemetry);
        mStateTime.startTime();
    }

    @Override
    public void start(){
        mStateTime.reset();
    }

    public void loop(){
        if(mStateTime.time() >= 4.0){
            intake.setPower(0.0);
            intake.close();
        }else{
            //intake.kickout();
            intake.getMotors()[0].getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.getMotors()[0].getMotor().setPower(-1.0);
            intake.open();
        }
        if(mStateTime.time() >= 1.0){
            flip.start();
        }
        intake.write();
        flip.write();
    }
}
