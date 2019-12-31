package org.firstinspires.ftc.teamcode.Odometry

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.State
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Mecanum_Drive
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2D
import org.openftc.revextensions2.ExpansionHubEx
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
    var odos : ThreeTrackingWheelLocalizer

    //Rotational straight line coefficients
    val kpr = 0.6
    val kir = 0.0
    val kdr = 0.0

    //Turn Coefficients
    val kpA = 0.39 //0.39
    val kiA = 0.008 //0.008
    val kdA = 0.2 //0.2

    //Rotational STRAFE coefficients
    val kprs = 0.6

    //Straight Line Coefficients
    val kpstr = 0.025 //0.055
    val kistr = 0.0 //0.0
    val kdstr = 9.25

    //Strafe PID coeffients
    val kpstf = 0.05
    val kistf = 0.0
    val kdstf = 10.0

    var preverror = 0.0
    var preverrorstr = 0.0
    var preverrorstf = 0.0
    var integralEstraight = 0.0
    var integralEdrift = 0.0
    var integralEstrafe = 0.0
    var rotintegralError = 0.0
    var rotprevheading = 0.0
    var vectorAngle = 0.0

    init {
        RevExtensions2.init()
        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        hub2 = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 1")
        odos = ThreeWheelTrackingLocalizer(hardwareMap)
        left = LeftWheel
        right = RightWheel
        strafe = StrafeWheel
        drive = Mecanum_Drive(hardwareMap, telemetry)
        left.reset()
        right.reset()
        strafe.reset()
    }

    private fun update() {
        val data = hub.bulkInputData
        left.update(data)
        right.update(data)
        strafe.update(data)
    }

    fun getTrackWidthAngle(): Double {
        val arcError = left.getDist() - right.getDist()
        val theta: Double
        if (arcError / TRACK_WIDTH <= 1.0 && arcError / TRACK_WIDTH >= -1.0) {
            theta = Math.asin(arcError / TRACK_WIDTH)
        } else {
            theta = 0.0
        }
        return -theta
    }

    fun ktrack(): Pose2d {
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
        val heading = getTrackWidthAngle()
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

    fun track(): Pose2d {
        update()
        val y = (left.getDist() + right.getDist()) / 2
        val x = strafe.getDist()
        val heading = getTrackWidthAngle()
        val p = Pose2d(x, y, heading)
        return p
    }

    var heading = 0.0

    fun GoTo(position: Pose2d) {
        val data = hub.bulkInputData
        val currentPos = odos.poseEstimate
        odos.update()

        val dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        val errorV = Vector2D(position.x - currentPos.y, position.y - currentPos.x)
        //val robotAngle = Math.atan2(errorV.x(), errorV.y())
        //robotAngle = Math.atan2(errorV.x(), errorV.y()) //todo: change this to error.x and y for goto()

        if(currentPos.heading >= Math.PI){
            heading = currentPos.heading
        }else{
            heading = -((2 * Math.PI ) - currentPos.heading)
        }
        vectorAngle = Math.atan2(errorV.x(), errorV.y()) - heading
        var yTrans = errorV.norm() * Math.cos(vectorAngle)
        var xTrans = errorV.norm() * Math.sin(vectorAngle)
        telemetry.addData("norm: ", errorV.norm())
        /*
        if (position.y - currentPos.x < 0.0) {
            yTrans = -yTrans
        } else if (position.x - currentPos.y < 0.0) {
            xTrans = -xTrans
        }

         */
        val error = (yTrans)//currentPos.y
        val straferror = (xTrans)//currentPos.x
        var rotheadingerror = position.heading - currentPos.heading
        telemetry.addData("taget:", position.toString())
        telemetry.addData("error x: ", xTrans)
        telemetry.addData("error y: ", yTrans)

        if(Math.abs(Math.toDegrees(rotheadingerror)) >= 180){
            if (rotheadingerror > 0) {
                rotheadingerror = -((Math.PI * 2) - abs(rotheadingerror))
            }
            else{
                rotheadingerror = ((Math.PI * 2) - abs(rotheadingerror))
            }
        }

        val propstr = error * kpstr
        val integralstr = integralEstraight * kistr
        val derivstr = kdstr * ((error - preverrorstr) / dt)

        val propstf = straferror * kpstf
        val integralstf = integralEstrafe * kpstf
        val derivstf = kdstf * ((straferror - preverrorstf) / dt)

        val powerstf = propstf + integralstf + derivstf
        val powerstr = propstr + integralstr + derivstr
        if (Math.abs(drive.getOverallPower()) < 0.1) {
            integralEstraight += error
        }else if(Math.abs(powerstf) < 0.2){
            integralEstrafe += straferror
        }
        preverrorstr = error
        preverrorstf = straferror

        var Drifterror = currentPos.heading

        val prop = Drifterror * kpr
        val integral = integralEdrift * kir
        val deriv = kdr * ((Drifterror - preverror) / dt)

        val propr = rotheadingerror * kpA
        val integralr = rotintegralError * kiA
        val derivr = kdA * ((rotheadingerror - rotprevheading)/ dt)

        val power = prop + integral + deriv
        val rotpower = propr + integralr + derivr
        if (Math.abs(power) < 0.3) {
            integralEdrift += error
        }
        preverror = error

        left.update(data)
        right.update(data)
        strafe.update(data)

        drive.setPower( Range.clip(-powerstr, -0.4, 0.4) , Range.clip(powerstf, -0.4, 0.4), Range.clip(rotpower, -0.4, 0.4))
        telemetry.addData("Strafe Power: ", powerstf)
        //drive.setPower( Range.clip(-powerstr, -0.3, 0.3), 0.0,0.0/*Range.clip(powerstf, -0.3, 0.3), Range.clip(rotpower, -0.3, 0.3)*/)
        //drive.setPower( /*Range.clip(powerstr, -0.3, 0.3)*/0.0 , Range.clip(powerstf, -0.3, 0.3), /*Range.clip(rotpower, -0.3, 0.3)*/ 0.0)
        //drive.setPower(/*Range.clip(powerstr, -0.3, 0.3)*/ 0.0, /*Range.clip(powerstf, -0.3, 0.3)*/0.0, Range.clip(rotpower, -0.3, 0.3))
        telemetry.addData("robotangle: ", vectorAngle)
        rotprevheading = currentPos.heading
        drive.write()
    }
}