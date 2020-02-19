package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;

@Autonomous(name = "Track Width Tuner", group = "Proto")
@Disabled
public class Track_Width_Tuner extends OpMode {
    Mecanum_Drive drive;
    ThreeWheelTrackingLocalizer odos;
    ExpansionHubEx hub;
    ExpansionHubEx hub2;
    double relativetrackwidth = 0.0;
    ArrayList<Double> exampleTrackWidths = new ArrayList<>();

    public void init(){
        RevExtensions2.init();
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        odos = new ThreeWheelTrackingLocalizer(hardwareMap);
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        odos.reset();
    }

    public void loop(){
        RevBulkData data = hub2.getBulkInputData();
        RevBulkData data2 = hub.getBulkInputData();
        drive.read(data);
        odos.update();
        odos.dataUpdate(data2);
        /*
        if(drive.angleWrap(drive.angleWrap(drive.getExternalHeading())) <= Math.toRadians(270.0) && drive.angleWrap(drive.getExternalHeading()) >= Math.toRadians(90)){
            relativetrackwidth = odos.getEstimatedPose().getHeading()/drive.angleWrap(drive.getExternalHeading());
            exampleTrackWidths.add(relativetrackwidth);
        }
         */
        if(drive.angleWrap(drive.angleWrap(drive.getExternalHeading())) != 0.0) {
            relativetrackwidth = odos.getEstimatedPose().getHeading() / drive.angleWrap(drive.getExternalHeading());
            exampleTrackWidths.add(relativetrackwidth);
        }
        drive.setPower(0.0,0.0,0.0);
        telemetry.addData("Raw Strafe: ", odos.getWheelPositions().get(2));
        telemetry.addData("Raw Forward1: ", odos.getWheelPositions().get(0));
        telemetry.addData("Raw Forward2: ", odos.getWheelPositions().get(1));
        telemetry.addData("IMU Angle: ", Math.toDegrees(drive.angleWrap(drive.getExternalHeading())));
        telemetry.addData("Heading RR!!!: ", Math.toDegrees(odos.getPoseEstimate().getHeading()));
        telemetry.addData("Heading Absolute!!!: ", Math.toDegrees(odos.getAbsoluteAngle()));
        telemetry.addData("Right Encoder: ", odos.getWheelPositions().get(1));
        telemetry.addData("Left Encoder: ", odos.getWheelPositions().get(0));
        telemetry.addData("Instantaneous Track Width: ", relativetrackwidth);
        telemetry.addData("Relative Track Width: ", average(exampleTrackWidths));

        drive.write();
    }

    private double average(ArrayList<Double> data){
        double sum = 0.0;
        for (double num : data){
            sum += (num / data.size());
        }

        return sum;
    }
}
