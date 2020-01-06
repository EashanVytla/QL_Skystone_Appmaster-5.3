package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

public class ThreeWheelTrackingLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 1.276; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.9235669; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -3.890925; //3.9625//3.92577 //4.08535032; // in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    private int[] positions = {0, 0, 0};

    public ThreeWheelTrackingLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("intake_left");
        rightEncoder = hardwareMap.dcMotor.get("intake_right");
        frontEncoder = hardwareMap.dcMotor.get("lift_2");
    }

    public void setPos(Pose2d pos){
        this.setPoseEstimate(pos);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public void dataUpdate(RevBulkData data){
        positions[0] = data.getMotorCurrentPosition(leftEncoder);
        positions[1] = data.getMotorCurrentPosition(rightEncoder);
        positions[2] = data.getMotorCurrentPosition(frontEncoder);
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(positions[0]),
                encoderTicksToInches(positions[1]),
                encoderTicksToInches(-positions[2])
        );
    }
}