package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Universal.Math.Pose;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

public class ThreeWheelTrackingLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 1.276; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.113085441402362; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4.875;// in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    private int[] positions = {0, 0, 0};
    public double heading;
    public double startheading;

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

    public void reset(){
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getAbsoluteAngle(){
        heading = angleWrap((encoderTicksToInches(positions[1]) - encoderTicksToInches(positions[0])) / LATERAL_DISTANCE);
        heading -= startheading;
        return heading;
    }

    public void poseSet(Pose2d pos){
        startheading = pos.getHeading();
        setPoseEstimate(pos);
    }

    public double getstrafeDiff(){
        return encoderTicksToInches(positions[2]) / FORWARD_OFFSET;
    }

    public Pose2d getEstimatedPose(){
        return new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(), getAbsoluteAngle());
    }

    private double angleWrap(double theta){
        return (theta + (2 * Math.PI)) % (2 * Math.PI);
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