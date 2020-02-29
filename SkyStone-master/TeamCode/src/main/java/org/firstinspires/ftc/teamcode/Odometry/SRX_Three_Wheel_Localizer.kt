package org.firstinspires.ftc.teamcode.Odometry

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.State
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Flipper
import org.firstinspires.ftc.teamcode.Mecanum_Drive
import org.firstinspires.ftc.teamcode.Universal.Math.Pose
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2D
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import org.openftc.revextensions2.RevExtensions2
import kotlin.math.abs

class SRX_Three_Wheel_Localizer(LeftWheel: SRX_Encoder, RightWheel: SRX_Encoder, StrafeWheel: SRX_Encoder, hardwareMap: HardwareMap, telemetry: Telemetry) {
    var left: SRX_Encoder
    var right: SRX_Encoder
    var strafe: SRX_Encoder
    val TRACK_WIDTH = 15.75
    val drive: Mecanum_Drive
    var prev_time = System.currentTimeMillis()
    var telemetry = telemetry

    var prevdistLeft = 0.0
    var prevdistRight = 0.0
    var prevdistStrafe = 0.0

    var hub: ExpansionHubEx
    var hub2: ExpansionHubEx
    var currentPos = Pose2d(0.0, 0.0, 0.0)
    var odos : ThreeWheelTrackingLocalizer

    //Turn Coefficients
    val kpA = 0.82 //0.39
    val kiA = 0.0
    val kdA = 0.07 //0.6

    //Straight Line Coefficients
    val kpstr = 0.05     //0.03
    val kistr = 0.0      //0.0
    val kdstr = 0.0125      //9.25

    //Strafe PID coeffients
    val kpstf = 0.13      //0.09
    val kistf = 0.002
    val kdstf = 0.02      //10.0

    //Rotational straight line coefficients
    val kpr = 0.6
    val kir = 0.0
    val kdr = 0.0

    //Rotational STRAFE coefficients
    val kprs = 0.6

    //feedrforward
    val kv = 0.02
    val ka = 0.005
    val kstatic = 0.0

    var preverror = 0.0
    var preverrorstr = 0.0
    var preverrorstf = 0.0
    var integralEstraight = 0.0
    var integralEdrift = 0.0
    var integralEstrafe = 0.0
    var rotintegralError = 0.0
    var rotprevheading = 0.0
    var vectorAngle = 0.0

    var pidstr : PIDFController
    var pidstf : PIDFController
    var pidr : PIDFController

    var first = true
    var primaryAngle = 0.0

    fun resetfirst(){
        first = true
    }

    //val strprofile : MotionProfile
    var flip : Flipper

    init {
        RevExtensions2.init()
        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        hub2 = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 1")
        odos = ThreeWheelTrackingLocalizer(hardwareMap)
        left = LeftWheel
        right = RightWheel
        strafe = StrafeWheel
        drive = Mecanum_Drive(hardwareMap, telemetry)
        flip = Flipper(hardwareMap, telemetry)
        left.reset()
        right.reset()
        strafe.reset()
        pidstr = PIDFController(PIDCoefficients(kpstr, kistr, kdstr), kv, ka, kstatic)
        pidstf = PIDFController(PIDCoefficients(kpstf, kistf, kdstf), kv, ka, kstatic)
        pidr = PIDFController(PIDCoefficients(kpA, kiA, kdA))


        /*
        strprofile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(45.0, 0.0, 0.0),
                25.0,
                40.0,
                100.0
        )
         */
    }
    var inverse = false

    fun inverse(){
        inverse = true
    }

    fun update(data : RevBulkData){
        odos.dataUpdate(data)
        odos.update()
    }

    fun reset(){
        pidstr.reset()
        pidstf.reset()
        pidr.reset()
    }

    fun update() {
        val data = hub.bulkInputData
        left.update(data)
        right.update(data)
        strafe.update(data)
    }

    fun getForwardDist() : Double{
        val data = hub.bulkInputData
        left.update(data)
        right.update(data)

        return (left.getDist() + right.getDist())/2
    }

    fun Lineartrack(): Pose2d {
        var data = hub.bulkInputData

        left.update(data)
        right.update(data)
        strafe.update(data)

        val leftv = Vector2D(0.0, left.getDist() - prevdistLeft)
        val rightv = Vector2D(0.0, right.getDist() - prevdistRight)
        val strafev = Vector2D(strafe.getDist() - prevdistStrafe, 0.0)
        val forwardv = leftv.normalize(rightv)
        var center = forwardv.normalize(strafev)
        telemetry.addData("Center: ", center.toString())
        val heading = odos.absoluteAngle
        var xTrans = center.norm() * Math.cos(heading)
        var yTrans = center.norm() * Math.sin(heading)
        if (center.y() < 0.0) {
            yTrans = -yTrans
        } else if (center.x() < 0.0) {
            xTrans = -xTrans
        }

        currentPos = Pose2d(currentPos.x + xTrans, currentPos.y + yTrans, heading)
        prevdistLeft = left.getDist()
        prevdistRight = right.getDist()
        prevdistStrafe = strafe.getDist()
        return currentPos
    }



    fun OutputRaw() {
        var data = hub.bulkInputData
        left.update(data)
        right.update(data)
        strafe.update(data)
        telemetry.addData("intake_right", right.getDist())
        telemetry.addData("intake_left", left.getDist())
        telemetry.addData("Strafe", strafe.getDist())
    }

    var heading = 0.0

    fun GoTo(position: Pose2d, strspeed : Double, stfspeed : Double, rspeed : Double){
        pidstr.setOutputBounds(-strspeed, strspeed)
        pidstf.setOutputBounds(-stfspeed, stfspeed)
        pidr.setOutputBounds(-rspeed, rspeed)


        currentPos = Pose2d(odos.poseEstimate.vec(), odos.absoluteAngle)

        if(currentPos.heading <= Math.PI){
            heading = currentPos.heading
        }else{
            heading = -((2 * Math.PI ) - currentPos.heading)
        }
        telemetry.addData("heading: ", heading)

        if (inverse) {
            pidr.targetPosition = -position.heading
            pidstr.targetPosition = position.y
            pidstf.targetPosition = -position.x
        }
        else{
            pidr.targetPosition = position.heading
            pidstr.targetPosition = position.y
            pidstf.targetPosition = position.x
        }


        //val state = strprofile[time]
        val powerstr = pidstr.update(currentPos.x/*, state.v, state.a*/)
        telemetry.addData("current pos: ", currentPos.toString())

        drive.centricSetPower(-powerstr, pidstf.update(-currentPos.y), pidr.update(heading), currentPos.heading)

        drive.write()
    }
}