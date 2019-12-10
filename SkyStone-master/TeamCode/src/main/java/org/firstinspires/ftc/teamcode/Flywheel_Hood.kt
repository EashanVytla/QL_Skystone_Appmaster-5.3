package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.Caching_Servo
import org.openftc.revextensions2.RevBulkData
import kotlin.math.PI

class Flywheel_Hood(servo : Caching_Servo) {
    var servo = servo

    fun setAngle(angle : Double){
        val newAngle = (PI / 2) - angle
        servo.setPosition(newAngle * 2 / PI)
    }

    fun getAngle() : Double{
        return (PI / 2) - (servo.getPosition() * (PI / 2))
    }

    fun write(){
        servo.write()
    }
}