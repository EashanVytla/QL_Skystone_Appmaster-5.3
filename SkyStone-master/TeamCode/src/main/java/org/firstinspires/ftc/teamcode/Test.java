//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
///*
// * Thanks to EasyOpenCV for the great API (and most of the example)
// *
// * Original Work Copright(c) 2019 OpenFTC Team
// * Derived Work Copyright(c) 2019 DogeDevs
// */
//@Autonomous(name = "Vision Doge.CV Template", group="Concept")
////@Disabled
//public class Test extends LinearOpMode {
//    private OpenCvCamera phoneCam;
//    private SkystoneDetectorV2 skyStoneDetector;
//
//    String skystonePosition = "none";
//    double xPosition;
//
//    @Override
//    public void runOpMode() {
//        /*
//         * Instantiate an OpenCvCamera object for the camera we'll be using.
//         * In this sample, we're using the phone's internal camera. We pass it a
//         * CameraDirection enum indicating whether to use the front or back facing
//         * camera, as well as the view that we wish to use for camera monitor (on
//         * the RC phone). If no camera monitor is desired, use the alternate
//         * single-parameter constructor instead (commented out below)
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View
//        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
//
//        /*
//         * Open the connection to the camera device
//         */
//        phoneCam.openCameraDevice();
//
//        /*
//         * Specify the image processing pipeline we wish to invoke upon receipt
//         * of a frame from the camera. Note that switching pipelines on-the-fly
//         * (while a streaming session is in flight) *IS* supported.
//         */
//        skyStoneDetector.useDefaults();
//        skyStoneDetector = new SkystoneDetectorV2();
//        phoneCam.setPipeline(skyStoneDetector);
//
//        /*
//         * Tell the camera to start streaming images to us! Note that you must make sure
//         * the resolution you specify is supported by the camera. If it is not, an exception
//         * will be thrown.
//         *
//         * Also, we specify the rotation that the camera is used in. This is so that the image
//         * from the camera sensor can be rotated such that it is always displayed with the image upright.
//         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//         * away from the user.
//         */
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            xPosition = skyStoneDetector.foundRectangle().x;
//
//            if (xPosition <= -2.0) { //TODO Tune these numbers
//                skystonePosition= "left";
//            } else if (xPosition > -2.0 && xPosition < 2.0) {
//                skystonePosition = "center";
//            } else if  (xPosition >= 2.0) {
//                skystonePosition = "right";
//            }
//
//            telemetry.addData("xPos", xPosition);
//            telemetry.addData("SkyStone Pos", skystonePosition);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        phoneCam.stopStreaming();
//        //webcam.closeCameraDevice();
//
//        while (opModeIsActive()) {
//            if (skystonePosition == "left") {
//                //Code
//            } else if (skystonePosition == "center") {
//                //Code
//            } else if (skystonePosition == "right") {
//                //Code
//            }
//        }
//    }
//}