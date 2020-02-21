package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.openftc.revextensions2.RevBulkData
import java.util.*

class Intake(hardwareMap: HardwareMap) {
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1.0
    var write_index = 0
    val open : Array<Caching_Servo>
    var time = ElapsedTime()
    var prevtime = 0.0

    init{
        motors = arrayOf(Caching_Motor(hardwareMap, "intake_left"), Caching_Motor(hardwareMap, "intake_right"))
        open = arrayOf(Caching_Servo(hardwareMap, "intake_left_jaw"), Caching_Servo(hardwareMap, "intake_right_jaw"))
        time.startTime()
    }

    fun start(){
        close()
    }

    enum class clamp{
        OPEN,
        CLOSE,
        IDOL,
    }

    var clampst = clamp.CLOSE
    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
    }

    fun write(){
        motors[write_index].write()
        open[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors[0].motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motors[1].motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motors[0].setPower(-power)
        motors[1].setPower(power)
    }

    fun setPosition(position : Double){
        open[0].setPosition(position)
        open[1].setPosition(1-position)
    }

    fun kickout(){ //RED
        motors[0].motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motors[1].motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        open[0].setPosition(0.0)
        open[1].setPosition(0.25)
        motors[1].setPower(1.0)
        write()
    }

    fun kickoutBLUE(){ //BLU
        motors[0].motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motors[1].motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        open[1].setPosition(0.7)
        open[0].setPosition(0.575)
        motors[1].setPower(1.0)
        write()
    }

    fun initIntake(){
        open[0].setPosition(0.5)
        open[1].setPosition(0.5)
    }

    fun open(){
        setPower(0.0)
        open[0].setPosition(0.3)
        open[1].setPosition(0.7)
    }

    fun close(){
        open[0].setPosition(0.525) //0.7 //todo: LOWER TO INCREASE TIGHTNESS
        open[1].setPosition(0.3) //0.25 //todo: HIGHER TO INCREASE TIGHTNESS
    }

    fun newState(clampState: clamp){
        clampst = clampState
        time.reset()
    }

    fun operate(g1 : Gamepad, g2: Gamepad){
        setPower((0.5 * g1.right_trigger) - (0.5 * g1.left_trigger))

        if(g2.left_bumper){
            newState(clamp.OPEN)
        }
        if(g1.b){
            newState(clamp.IDOL)
        }


        if(clampst == clamp.OPEN){
            val wait_time = when{
                FlipperV2.rcase != 0 -> 3.0
                else -> 2.0
            }
            if (time.time() <= wait_time) {
                open()
            } else {
                newState(clamp.IDOL)
            }
        }else if(clampst == clamp.IDOL){
            if(g1.b){
                open()
            }else{
                close()
            }

        }

        write()
    }
}