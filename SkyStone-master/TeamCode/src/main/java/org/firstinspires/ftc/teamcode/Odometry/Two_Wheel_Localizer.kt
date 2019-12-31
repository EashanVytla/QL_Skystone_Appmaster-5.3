package org.firstinspires.ftc.teamcode.Odometry

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.FresnelIntegrals
import org.firstinspires.ftc.teamcode.LynxOptimizedI2cFactory
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.lang.StrictMath.cos
import kotlin.math.abs
import kotlin.math.sin

class Two_Wheel_Localizer(ex : Dead_Wheel, ey : Dead_Wheel, hub : ExpansionHubEx) {
    var ex = ex
    var ey = ey
    var prev_x_dist = 0.0
    var prev_y_dist = 0.0
    val imu : BNO055IMU

    val TRACK_WIDTH = 18

    var prev_heading = 0.0
    var prev_velo = 0.0

    var pos = Pose2d(0.0, 0.0, 0.0)

    init{
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
    }

    fun update(data : RevBulkData, dt : Long) : Pose2d{
        ex.update(data)
        ey.update(data)
        val orientation = imu.angularOrientation
        val rc_offset = Vector2d((ex.dist - prev_x_dist) - ((TRACK_WIDTH / 2) * orientation.firstAngle), ey.dist - prev_y_dist)
        val accel = -(2*prev_velo) / dt
        val offset : Vector2d
        if (accel != 0.0) {
            offset = constantAccelOdometryUpdate(rc_offset, accel, dt).rotated(prev_heading)
            prev_velo += accel * dt
        }
        else{
            val w = (orientation.firstAngle - prev_heading)
            offset = constantVeloUpdate(rc_offset, w).rotated(prev_heading)
        }
        pos = pos.plus(Pose2d(offset, orientation.firstAngle - prev_heading))
        prev_heading = orientation.firstAngle.toDouble()
        return pos
    }

    fun constantVeloUpdate(offset : Vector2d, w : Double): Vector2d{
        val (sineTerm, cosTerm) = if (abs(w) < 0.001) {
            1.0 - w * w / 6.0 to w / 2.0
        } else {
            sin(w) / w to (1 - cos(w)) / w
        }

        return Vector2d(
                sineTerm * offset.x - cosTerm * offset.y,
                cosTerm * offset.x + sineTerm * offset.y
        )
    }

    fun constantAccelOdometryUpdate(offset : Vector2d, a : Double, dt : Long) : Vector2d {
        var u = 0.0
        var s = 0.0
        var v = 0.0
        if (a > 0){
            val cosTerm = Math.cos(Math.pow(prev_velo, 2.0) / (2 * a))
            val sinTerm = Math.sin(Math.pow(prev_velo, 2.0) / (2 * a))
            val fTerm = (a * dt + prev_velo) / Math.sqrt(a * Math.PI)
            val fCosTerm = FresnelIntegrals.C(fTerm)
            val fSinTerm = FresnelIntegrals.S(fTerm)

            u = dt * Math.sqrt(Math.PI) * (cosTerm * fCosTerm + sinTerm * fSinTerm) / Math.sqrt(a)
            s = dt * Math.sqrt(Math.PI) * (sinTerm * fCosTerm - cosTerm * fSinTerm) / Math.sqrt(a)
            v = dt * Math.sqrt(Math.PI) * (cosTerm * fSinTerm - sinTerm * fCosTerm) / Math.sqrt(a)
        }
        else if (a < 0){
            val cosTerm = Math.cos(Math.pow(prev_velo, 2.0) / (2 * a))
            val sinTerm = Math.sin(Math.pow(prev_velo, 2.0) / (2 * a))
            val fTerm = (a * dt + prev_velo) / Math.sqrt(Math.abs(a) * Math.PI)
            val fCosTerm = FresnelIntegrals.C(-fTerm)
            val fSinTerm = FresnelIntegrals.S(-fTerm)

            u = Math.sqrt(Math.PI) * (cosTerm * fCosTerm - sinTerm * fSinTerm) / (Math.sqrt(Math.abs(a)) * dt)
            s = Math.sqrt(Math.PI) * (sinTerm * fCosTerm + cosTerm * fSinTerm) / (Math.sqrt(Math.abs(a)) * dt)
            v = -Math.sqrt(Math.PI) * (cosTerm * fSinTerm + sinTerm * fCosTerm) / (Math.sqrt(Math.abs(a)) * dt)
        }
        return Vector2d(u * offset.x + s * offset.y, v * offset.x + u * offset.y)
    }
}