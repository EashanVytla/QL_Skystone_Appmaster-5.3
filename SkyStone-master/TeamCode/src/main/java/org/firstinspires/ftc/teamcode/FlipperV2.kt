package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

class FlipperV2(hardwareMap : HardwareMap, intake : Intake) {
    var servos = arrayOf(Caching_Servo(hardwareMap, "AlignerTest"), Caching_Servo(hardwareMap, "clamp"), Caching_Servo(hardwareMap, "Deposit"), Caching_Servo(hardwareMap, "turn"))
    var intake = intake
    var write_index = 0

    enum class flipState{
        STATE_FLIP,
        STATE_CLAMP,
        STATE_IDLE,
        STATE_DEPOSIT,
        STATE_RELEASE,
        STATE_INTERMEDIATE
    }

    var mStateTime = ElapsedTime()
    var state = flipState.STATE_IDLE

    fun write(){
        servos.map{
            it.write()
        }
    }

    fun init(){
        servos[0].setPosition(0.3)
        servos[1].setPosition(0.7)
        servos[2].setPosition(0.9)
        servos[3].setPosition(0.5)
        write() //change to 4x write when making refresh rate optimization
    }

    fun start(){
        servos[0].setPosition(1.0)
        servos[1].setPosition(0.7)
        servos[2].setPosition(0.0)
        servos[3].setPosition(0.5)
        write() //change to 4x write when making refresh rate optimization
    }

    fun flip_up(){
        servos[0].setPosition(0.5)
    }

    fun flip_down(){
        servos[0].setPosition(0.0)
    }

    fun clamp(){
        servos[1].setPosition(0.95)
    }

    fun release(){
        servos[1].setPosition(0.7)
    }

    fun deposit(){
        servos[2].setPosition(0.0)
    }

    fun retract(){
        servos[2].setPosition(0.9)
    }

    fun hover(){
        servos[2].setPosition(0.3)
    }

    fun newState(inputState : flipState){
        state = inputState
        mStateTime.reset()
    }

    fun runFSM(g : Gamepad){
        if (state == flipState.STATE_FLIP){
            flip_up()
            if (mStateTime.time() >= 1.0){
                newState(flipState.STATE_CLAMP)
            }
        }
        else if (state == flipState.STATE_CLAMP){
            clamp()
            if (mStateTime.time() >= 1.0){
                newState(flipState.STATE_IDLE)
            }
        }
        else if (state == flipState.STATE_IDLE){
            flip_down()
            retract()
            intake.close()
            if (g.dpad_down){
                newState(flipState.STATE_DEPOSIT)
            }
            if (g.dpad_left){
                newState(flipState.STATE_INTERMEDIATE)
            }
        }
        else if (state == flipState.STATE_DEPOSIT){
            deposit()
            if (g.dpad_down){
                newState(flipState.STATE_RELEASE)
            }
        }
        else if (state == flipState.STATE_INTERMEDIATE){
            hover()
            if (g.dpad_down){
                newState(flipState.STATE_DEPOSIT)
            }
            else if (g.dpad_up){
                newState(flipState.STATE_IDLE)
            }
        }
        else if (state == flipState.STATE_RELEASE){
            release() //might need a position to clear entirely from the stack
            hover()
            if (g.dpad_up){
                newState(flipState.STATE_IDLE)
            }
        }
    }

    fun operate(g : Gamepad){
        if (g.y){
            intake.open()
            newState(flipState.STATE_FLIP)
        }
        runFSM(g)
        write()
    }
}