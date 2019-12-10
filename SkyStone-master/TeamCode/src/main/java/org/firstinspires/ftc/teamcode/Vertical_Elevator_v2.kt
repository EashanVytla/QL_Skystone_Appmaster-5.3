package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Universal.Motion.MotionProfile
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Vertical_Elevator_v2(map : HardwareMap, t : Telemetry){
    var motors : Array<DcMotor>

    val UPDATE_RATE = 1
    var write_index = 0

    var telemetry = t

    var slide_height = 0

    var query = 0.0

    enum class slideState{
        STATE_RAISE,
        STATE_DROP,
        STATE_IDLE
    }

    var mSlideState = slideState.STATE_IDLE

    init{
        motors = arrayOf(map.dcMotor.get("lift_1"), map.dcMotor.get("lift_2"))
        motors[1].direction = DcMotorSimple.Direction.REVERSE
        //motors[0].motor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun read(data : RevBulkData){
        slide_height = (data.getMotorCurrentPosition(motors[0]) + data.getMotorCurrentPosition(motors[1])) / 2
    }

    fun setPower(power : Double){
        motors.map{
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        query = power
    }

    fun write(){
        motors[write_index].power = query
        write_index = (write_index + 1) % 2
    }

    fun newState(s : slideState){
        mSlideState = s
    }

    fun setTargetPosition(target : Int){
        motors[0].targetPosition = target
        motors[1].targetPosition = target//motors[0].getCurrentPosition() //test to see if pos1 = -pos2, recommended test is to output encoder positions from both motors and analyze

        if (motors[0].mode != DcMotor.RunMode.RUN_TO_POSITION){
            motors[0].mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        if (motors[1].mode != DcMotor.RunMode.RUN_TO_POSITION){
            motors[1].mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        motors.map{
            it.power = 1.0
        }
    }

    fun operate(g : Gamepad){
        if (g.y){
            newState(slideState.STATE_RAISE)
        }
        else if (g.a){
            newState(slideState.STATE_DROP)
        }
        else if (g.b){
            newState(slideState.STATE_DROP)
        }

        if (mSlideState == slideState.STATE_RAISE){
            setTargetPosition(-925)
            if (abs(slide_height + 925) < 25){
                setPower(0.0)
                newState(slideState.STATE_IDLE)
            }
        }
        else if (mSlideState == slideState.STATE_DROP){
            setTargetPosition(0)
            if (slide_height > -25){
                setPower(0.0)
                newState(slideState.STATE_IDLE)
            }
        }
        else if (mSlideState == slideState.STATE_IDLE){
            setPower(g.right_stick_y.toDouble())
        }
        else{
            telemetry.addData("You fucked up", motors[0].getCurrentPosition())
        }
        write()
    }
}