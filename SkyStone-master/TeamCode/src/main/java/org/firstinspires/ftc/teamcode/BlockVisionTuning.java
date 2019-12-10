package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_WIDTH;

@TeleOp(name = "Vision")
public class BlockVisionTuning extends OpMode {
    //FtcDashboard dashboard;
    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    public void init() {
        //dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        //telemetry = new MultipleTelemetry(super.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        //dashboard.sendImage(pipeline.imageSend);

        telemetry.addData("Stone Pos", pipeline.getSkyPos());
        telemetry.addData("Stonesize0", pipeline.stoneSizes[0]);
        telemetry.addData("Stonesize1", pipeline.stoneSizes[1]);
        telemetry.addData("Stonesize2", pipeline.stoneSizes[2]);
        telemetry.update();

    }

}
