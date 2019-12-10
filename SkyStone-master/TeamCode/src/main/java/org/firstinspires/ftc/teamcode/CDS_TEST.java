package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "CDS_TEST",group = "PROTO")
public class CDS_TEST extends OpMode {
    DistanceSensor sensorDistance;
    int rcase;

    public void init(){
        sensorDistance = hardwareMap.get(DistanceSensor.class, "cds");
    }

    public void loop(){
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        if(sensorDistance.getDistance(DistanceUnit.CM) >= 6.5 && sensorDistance.getDistance(DistanceUnit.CM) <= 9.25){
            //Case regular
            //6.75 - 7.5
            rcase = 0;
        }else if(sensorDistance.getDistance(DistanceUnit.CM) >= 5.45 && sensorDistance.getDistance(DistanceUnit.CM) <= 6.45){
            //Case left
            //5.45 - 6
            rcase = 1;
        }else if(sensorDistance.getDistance(DistanceUnit.CM) >= 9.5 && sensorDistance.getDistance(DistanceUnit.CM) <= 12.0){
            //Case right
            //9.5 - 10.5
            rcase = 2;
        }else{
            rcase = 3;
        }
        telemetry.addData("Case:", rcase);
    }
}
