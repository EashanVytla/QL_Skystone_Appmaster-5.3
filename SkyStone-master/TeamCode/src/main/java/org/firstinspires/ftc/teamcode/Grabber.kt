package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap

class Grabber(hardwareMap: HardwareMap) {
    var SClamp: Caching_Servo
    var SArm: Caching_Servo

    init{
        SClamp = Caching_Servo(hardwareMap, "sclamp")
        SArm = Caching_Servo(hardwareMap, "sarm")
        initialize()
    }

    fun write(){
        SClamp.write()
        SArm.write()
    }

    fun clamp(){
        SClamp.setPosition(0.1) //Clamp
    }

    fun unclamp(){
        SClamp.setPosition(0.325) //Unclamp
    }

    fun goDown(){
        SArm.setPosition(0.415) //Down Position
    }

    fun goUp(){
        SArm.setPosition(0.0) //All the way up
    }

    fun liftCrossing(){
        SArm.setPosition(0.113)
    }

    fun partial(){
        SArm.setPosition(0.3) //Partial
    }

    fun deposit(){
        SArm.setPosition(0.157)
    }

    fun collapse(){
        SClamp.setPosition(0.0)
    }

    fun initialize(){
        SClamp.setPosition(0.1)
        SArm.setPosition(0.0)
        write()
        write()
    }
}