package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevExtensions2

class PID(hardwareMap: HardwareMap, telemetry: Telemetry) {
    var leftWheel: Dead_Wheel
    var rightWheel: Dead_Wheel
    var strafeWheel: Dead_Wheel

    var drive: Mecanum_Drive

    var pid : PIDFController

    //All PID variables
    var prev_time = System.currentTimeMillis()
    var headingerror = 0.0
    var prevheading = 0.0
    var preverror = 0.0
    var preverrorstr = 0.0
    var error = 0.0
    var drifterror = 0.0
    var integralError = 0.0
    var integralEstraight = 0.0
    var integralEdrift = 0.0
    var dt = 0.0
    var angle = 0.0
    var targetangle = 0.0

    //Rotational straight line coefficients
    val kpr = 0.3
    val kir = 0.0
    val kdr = 0.0

    //Straight Line Coefficients
    val kp = 0.01 //0.02 //0.1
    val ki = 0.002 //0.0
    val kd = 0.2 //0.2

    //Turn Coefficients
    val kpA = 0.39//0.35625
    val kiA = 0.008//0.005
    val kdA = 0.2

    //TurnPlatform Coefficients
    val kpP = 1.0


    val TRACK_WIDTH = 14.85006069

    //var imu: BNO055IMU
    private var hub: ExpansionHubEx
    private val hub2: ExpansionHubEx

    init {
        RevExtensions2.init()
        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        hub2 = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 1")

        leftWheel = Dead_Wheel(MA3_Encoder("a3", hardwareMap, 0.495))
        rightWheel = Dead_Wheel(MA3_Encoder("a4", hardwareMap, 1.365))
        strafeWheel = Dead_Wheel(MA3_Encoder("a1", hardwareMap, 2.464))
        rightWheel.encoder.reverse()
        strafeWheel.encoder.reverse()
        val data = hub.bulkInputData
        val data2 = hub2.bulkInputData
        //leftWheel.encoder.calibrate(data)
        //rightWheel.encoder.calibrate(data2)
        //strafeWheel.encoder.calibrate(data)
        drive = Mecanum_Drive(hardwareMap, telemetry)
        pid = PIDFController(PIDCoefficients(kp, ki, kd))
        pid.setOutputBounds(-0.5, 0.5)

        /*
        leftWheel.setBehavior(1.5385, -0.319237) //1.5144 0.0361262

        rightWheel.setBehavior(1.5385, -0.319237) //1.5204 -0.00305571

        strafeWheel.setBehavior(1.53642, 0.0) //1.50608 -0.221642
        */

        /*
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

         */
    }

    fun getTrackWidthAngle() : Double {
        var arcError = getleftDist() - getrightDist()
        var theta = Math.asin(arcError/TRACK_WIDTH)
        return theta
    }

    fun reset(){
        pid.reset()
        integralError = 0.0
        integralEstraight = 0.0
    }

    fun straight(target : Double, forwardDist : Double, telemetry: Telemetry){
        val data = hub.bulkInputData
        val data2 = hub2.bulkInputData

        //pid.targetPosition = target

        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        angle = drive.angleWrap(drive.getExternalHeading())
        error = target + forwardDist


        val propstr = error * kp
        val integralstr = integralEstraight * ki
        val derivstr = ((error - preverrorstr) * kd) / dt

        val powerstr = propstr + integralstr + derivstr
        if (Math.abs(powerstr) < 0.15) {
            integralEstraight += error
        }
        preverrorstr = error


        var Drifterror = getTrackWidthAngle()

        //pid.targetPosition = target

        val prop = Drifterror * kpr
        val integral = integralEdrift * kir
        val deriv = ((Drifterror - preverror) * kdr) / dt

        val power = prop + integral + deriv
        if (Math.abs(power) < 0.3) {
            integralEdrift += error
        }
        preverror = error

        leftWheel.update(data)
        rightWheel.update(data2)
        strafeWheel.update(data)
        //todo: FIRST CHECK WHAT VALUES THIS IS GIVING. IF IT IS NOT CHANGING THEN JUST PASS IT IN!!!!

        telemetry.addData("Left Wheel: ", getleftDist())
        telemetry.addData("Right Wheel: ", getrightDist())
        telemetry.addData("Strafe Wheel: ", getStrafeDist())
        telemetry.addData("Angle: ", getTrackWidthAngle())
        telemetry.addData("PowerRot: ", power)

        //drive.setPower(pid.update(-forwardDist), 0.0, Range.clip(power, -0.4, 0.4))
        if(Math.abs(getleftDist() - getrightDist()) <= 1.5){
            drive.setPower(Range.clip(powerstr, -0.5, 0.5), 0.0, Range.clip(power, -0.4, 0.4))
        }else{
            drive.setPower(Range.clip(powerstr, -0.5, 0.5), 0.0, 0.0)
        }
        //drive.setPower(Range.clip(powerstr, -0.5, 0.5), 0.0, 0.0)

        drive.write()
    }


    fun getleftDist(): Double{
        return leftWheel.getDistance() * (23 / 37.678)
    }

    fun getrightDist() : Double{
        return rightWheel.getDistance() * (23 / 37.678)
    }

    fun getStrafeDist(): Double {
        return strafeWheel.getDistance() * 7.0 / 17.536
    }

    fun getforwardDist() : Double{
        return rightWheel.getDistance() * (23 / 37.678)
    }
}