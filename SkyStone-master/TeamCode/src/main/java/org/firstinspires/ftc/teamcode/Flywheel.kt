package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2
import org.openftc.revextensions2.RevBulkData
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.pow

class Flywheel(hardwareMap: HardwareMap, name : String, servo_name : String) {
    val motor : Caching_Motor
    val hood : Flywheel_Hood
    var controller = PDF_Controller(PDF_Coefficients(1.0, 1.0, 1.0 / 1780.0))

    var prev_pos = 0
    var prev_time = System.currentTimeMillis()

    val update_frequency = 0.1

    val FLYWHEEL_DIAMETER = 4
    val HEIGHT = 10

    init{
        motor = Caching_Motor(hardwareMap, name)
        hood = Flywheel_Hood(Caching_Servo(hardwareMap, servo_name))
    }

    fun getVelocity() : Double{
        val velo = countsToDegrees(motor.getCurrentPosition() - prev_pos) / (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        prev_pos = motor.getCurrentPosition()
        return velo
    }

    fun full_send(){
        setVelocity(1780.0)
    }

    private fun countsToDegrees(counts : Int) : Double{
        return (counts.toDouble() / 103.6) * 2 * PI
    }

    fun setVelocity(velocity : Double){
        motor.setPower(controller.update(velocity, getVelocity()))
    }

    fun read(data : RevBulkData){
        motor.read(data)
    }

    fun write(){
        motor.write(update_frequency)
    }

    fun solveVeloAngle(dist : Double) : Double{ //return time to target in milliseconds
        val velo = getVelocity() * FLYWHEEL_DIAMETER
        if (velo != 0.0) {
            val top = (16 * dist.pow(2)) + (HEIGHT * velo.pow(2))
            val bottom = dist * ((16 * dist) + velo.pow(2))

            hood.setAngle(atan(top / bottom))
            return velo * cos(atan(top / bottom))
        }

        return -1.0
    }

    fun estimateVeloAngle(dist : Double) : Double{
        val velo = getVelocity() * FLYWHEEL_DIAMETER
        if (velo != 0.0) {
            val top = (16 * dist.pow(2)) + (HEIGHT * velo.pow(2))
            val bottom = dist * ((16 * dist) + velo.pow(2))

            //hood.setAngle(atan(top / bottom))
            return velo * cos(atan(top / bottom))
        }

        return -1.0
    }

    fun cooldown(){
        setVelocity(0.0)
    }

    fun fire(dist : Double) : Double{
        full_send()
        return solveVeloAngle(dist)
    }

    fun aim_fire(target : Vector2, velo : Vector2, pos : Vector2) : Double{
        full_send()
        val timeToTarget = estimateVeloAngle(target.distanceToVector(pos))
        velo.scalarMultiply(timeToTarget)
        return solveVeloAngle(target.compound(velo).distanceToVector(pos))
    }
}