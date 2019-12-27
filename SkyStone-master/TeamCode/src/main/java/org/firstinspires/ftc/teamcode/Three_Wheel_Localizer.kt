package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs
import kotlin.math.sin

class Three_Wheel_Localizer(e0 : Dead_Wheel, e1 : Dead_Wheel, e2 : Dead_Wheel) {
    var ex1 = e0
    var ex2 = e1
    var ey = e2

    var prev_x1_dist = 0.0
    var prev_x2_dist = 0.0
    var prev_y_dist = 0.0

    val TRACK_WIDTH = 14.85006069
    val STRAFE_WIDTH = 4.362

    var prev_heading = 0.0
    var prev_velo = 0.0

    var heading = 0.0

    val WHEEL_VARIANCE = 0.5

    var pos = Pose2d(0.0, 0.0, 0.0)

    init{
        ex2.encoder.reverse()
        ey.encoder.reverse()
    }

    fun angleWrap(angle : Double) : Double{
        return (angle + (2 * Math.PI)) % (2 * Math.PI)
    }

    fun update(data : RevBulkData, dt : Long) : Pose2d{
        ex1.update(data)
        ex2.update(data)
        ey.update(data)

        val x_offset = ((ex1.getDistance() - prev_x1_dist) + (ex2.getDistance() - prev_x2_dist)) / 2.0
        val dTheta = ((ex1.getDistance() - prev_x1_dist) - (ex2.getDistance() - prev_x2_dist)) / (TRACK_WIDTH + (ex1.POC * WHEEL_VARIANCE) + (ex2.POC * WHEEL_VARIANCE))
        val y_dist = (ey.getDistance() - prev_y_dist) - (dTheta * STRAFE_WIDTH)
        val rc_offset = Vector2d(x_offset, y_dist)
        val accel = (2 * (dTheta - (prev_velo * dt))) / Math.pow(dt.toDouble(), 2.0)
        val offset : Vector2d

        if (accel != 0.0) {
            offset = constantAccelOdometryUpdate(rc_offset, accel, dt).rotated(prev_heading)
            prev_velo += accel * dt
        }
        else{
            offset = constantVeloUpdate(rc_offset, dTheta).rotated(prev_heading)
        }

        pos = pos.plus(Pose2d(offset, dTheta))
        prev_heading = heading
        heading = angleWrap(heading + dTheta)
        prev_x1_dist = ex1.getDistance()
        prev_x2_dist = ex2.getDistance()
        prev_y_dist = ey.getDistance()
        return pos
    }

    fun constantVeloUpdate(offset : Vector2d, w : Double): Vector2d{
        val (sineTerm, cosTerm) = if (abs(w) < 0.001) {
            1.0 - w * w / 6.0 to w / 2.0
        } else {
            sin(w) / w to (1 - StrictMath.cos(w)) / w
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