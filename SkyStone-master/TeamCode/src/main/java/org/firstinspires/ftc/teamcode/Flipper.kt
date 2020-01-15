package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.openftc.revextensions2.RevBulkData
import java.util.*

class Flipper(h : HardwareMap, telemetry: Telemetry){
    var flipper : Caching_Servo
    var clamp : Caching_Servo
    var deposit : Caching_Servo
    var turn : Caching_Servo
    var leftpm : Caching_Servo
    var rightpm : Caching_Servo
    var time = ElapsedTime()
    var t = telemetry
    var grabbed = false
    var turnPos = 0.5
    var sensorDistance: DistanceSensor
    var dist = 0.0
    var knocker = false

    var previous = false
    var previouscap = false
    var previousclamp = false
    var prevknocker = false

    val capClamp : Caching_Servo
    val capDeposit : Caching_Servo

    var clamped = false
    var prev_sequence = -1
    var lock = false

    companion object {
        var capped = false
        var rcase = 0
        const val case_right_turn_value = 0.795
        const val case_left_turn_value = 0.17
        const val case_center_turn_value = 0.52

        const val handshake_flip_position = 0.45 //THIS IS GOING BACKWARDS 1 -> 0

        const val turnPos_IDOL = 0.4935
        const val flipperPos_IDOL = 0.975 //THIS IS GOING BACKWARDS 1 -> 0
        const val DepositPos_IDOL = 0.1 //THIS IS GOING BACKWARDS 1 -> 0

        const val DepositPos = 1.0
        const val Deposit_Clearance_DROPPING_Block = 0.85
        const val Deposit_Clearance_HANDSHAKE = .15
    }
    var Flipper_Midway_REALLIGN = 0.0 //THIS IS GOING BACKWARDS 1 -> 0

    fun clamp(){
        clamp.setPosition(1.0)
    }

    fun unclamp(){
        clamp.setPosition(0.7)
    }

    enum class flip_state{
        STATE_CLAMP,
        STATE_FLIP,
        STATE_DEPOSIT,
        STATE_REALLIGN,
        STATE_DROP,
        STATE_IDLE
    }

    var betterFlipState = flip_state.STATE_IDLE

    fun getFlipState() : flip_state{
        return betterFlipState
    }

    fun flipDown(){
        flipper.setPosition(flipperPos_IDOL)
    }

    init{
        flipper = Caching_Servo(h, "AlignerTest")
        clamp = Caching_Servo(h,"clamp")
        deposit = Caching_Servo(h,"Deposit")
        turn = Caching_Servo(h,"turn")
        rightpm = Caching_Servo(h, "rightpm")
        leftpm = Caching_Servo(h, "leftpm")
        capClamp = Caching_Servo(h, "cclamp")
        capDeposit = Caching_Servo(h, "cdeposit")
        sensorDistance = h.get(DistanceSensor::class.java, "cds")
    }

    fun write(){
        flipper.write()
        clamp.write()
        deposit.write()
        turn.write()
        leftpm.write()
        rightpm.write()
        capClamp.write()
        capDeposit.write()
    }

    fun start(){
        turn.setPosition(turnPos_IDOL)
        deposit.setPosition(0.025)
        unclamp()
        deposit.setPosition(DepositPos_IDOL)
        time.startTime()
        flipper.setPosition(flipperPos_IDOL)
        capClamp.setPosition(1.0)
        capDeposit.setPosition(0.0)
        write()
    }

    fun initialize(){
        clamp()
        deposit.setPosition(0.0)
        turn.setPosition(turnPos_IDOL)
        leftpm.setPosition(0.3)
        rightpm.setPosition(0.75)
        capClamp.setPosition(1.0)
        capDeposit.setPosition(0.0)
        write()
    }

    fun read(){
        dist = sensorDistance.getDistance(DistanceUnit.CM)
    }

    fun getCDDist() : Double{
        dist = sensorDistance.getDistance(DistanceUnit.CM)
        return dist
    }

    fun IntakeFeedback() : Boolean{
        if((dist >= 6.5 && dist <= 9.25) || dist >= 13.0 && dist <= 16.0){
            return true
        }else{
            return false
        }
    }

    fun newState(flipState: flip_state){
        betterFlipState = flipState
        time.reset()

    }

    fun getCase() : Int{
        read()
        if ((dist >= 6.5 && dist <= 9.25) || (dist >= 13.0)) {
            //Case regular
            //6.75 - 7.5
            rcase = 0
        } else if (dist >= 5.45 && dist <= 6.45) {
            //Case left
            //5.45 - 6
            rcase = 1
        } else if (dist >= 9.5 && dist <= 12.0) {
            //Case right
            //9.5 - 10.5
            rcase = 2
        }
        return rcase
    }

    fun getRCase() : Int{
        return rcase
    }

    fun flip(){
        unclamp()
        if(time.time() >= 0.5){ //Wait for setup procedure before flipping
            turn.setPosition(turnPos)
            flipper.setPosition(handshake_flip_position)
            if(time.time() >= 1.3){
                newState(flip_state.STATE_CLAMP)
            }
        }
    }

    fun grabPlatform(){
        leftpm.setPosition(1.0)
        rightpm.setPosition(0.0)
        grabbed = true

        write()
    }

    fun isGrabbed() : Boolean{
        return grabbed
    }

    fun startKnocker(){
        leftpm.setPosition(0.625)
        rightpm.setPosition(0.325)

        knocker = true

        write()
    }

    fun resetPlatform(){
        leftpm.setPosition(0.2)
        rightpm.setPosition(0.75)
        grabbed = false
        knocker = false
        write()
    }

    private fun isPress(clicked : Boolean, previous : Boolean) : Boolean{
        return clicked && !previous
    }


    fun clampcap(){
        capClamp.setPosition(1.0)
        write()
    }


    fun operate(g1: Gamepad, g2 : Gamepad){
            if(g1.right_bumper){
                newState(flip_state.STATE_DROP)
            }
            if(g2.b){
                newState(flip_state.STATE_IDLE)
            }
        /*
            if(isPress(g1.y, prevknocker)){
                if(knocker){
                    resetPlatform()
                    knocker = false
                }else{
                    startKnocker()
                    knocker = true
                }
            }
            prevknocker = g1.y

         */
            if(isPress(g1.a, previous)){
                if(grabbed){
                    resetPlatform()
                    grabbed = false
                }else{
                    grabPlatform()
                    grabbed = true
                }
            }
            previous = g1.a

            if(g2.right_bumper){
                newState(flip_state.STATE_DEPOSIT)
            }

            if(isPress(g2.dpad_right, previousclamp)){
                if(!clamped){
                    capClamp.setPosition(1.0)
                    clamped = true
                }else{
                    capClamp.setPosition(0.9)
                    clamped = false
                }
            }
            previousclamp = g2.dpad_right

            if(isPress(g2.dpad_left, previouscap)){
                if(!capped){
                    capDeposit.setPosition(1.0)
                    capped = true
                }else{
                    capDeposit.setPosition(0.0)
                    capped = false
                }
            }
            previouscap = g2.dpad_left

            if(g2.left_bumper){
                if(getCase() == 0) {
                    //Case regular
                    turnPos = case_center_turn_value
                    newState(flip_state.STATE_FLIP)
                }
                if(getCase() == 2){
                    //Case Right
                    Flipper_Midway_REALLIGN = 0.7
                    turnPos = case_right_turn_value
                    newState(flip_state.STATE_REALLIGN)
                }
                if(getCase() == 1){
                    //Case Left
                    Flipper_Midway_REALLIGN = 0.75
                    turnPos = case_left_turn_value
                    newState(flip_state.STATE_REALLIGN)
                }
            }

        /*
        if(g2.right_trigger >= 0.5){
            turnPos = case_right_turn_value
            newState(flip_state.STATE_REALLIGN)
        }
        if(g2.left_trigger >= 0.5){
            turnPos = case_left_turn_value
            newState(flip_state.STATE_REALLIGN)
        }
         */

        if (betterFlipState == flip_state.STATE_FLIP){
            unclamp()
            if(time.time() >= 0.3){ //Wait for setup procedure before flipping
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
                if(time.time() >= 1.0){
                    newState(flip_state.STATE_CLAMP)
                }
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp()
            if (time.time() >= 0.5) {   //Wait for 1 second for the flipper to go back
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_IDLE){
            turnPos = turnPos_IDOL
            flipper.setPosition(flipperPos_IDOL)
            turn.setPosition(turnPos)
            deposit.setPosition(DepositPos_IDOL)
            //lock = true
            clamp()
        }else if(betterFlipState == flip_state.STATE_DEPOSIT){
            deposit.setPosition(DepositPos)
        }else if(betterFlipState == flip_state.STATE_DROP) {
            unclamp()
            if(time.time() >= 0.3){
                deposit.setPosition(Deposit_Clearance_DROPPING_Block)
            }
        }else if(betterFlipState == flip_state.STATE_REALLIGN){
            unclamp()
            if(time.time() >= 0.5){ //0.5
                flipper.setPosition(Flipper_Midway_REALLIGN)
            }
            if(time.time() >= 1.0){ //0.6
                turn.setPosition(((turnPos-0.5)/2) + 0.5)
                //flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 1.8){
                turn.setPosition(turnPos)
                if(time.time() >= 1.9){
                    flipper.setPosition(handshake_flip_position)
                }
            }
            if(time.time() >= 2.5){
                clamp()
            }
            if(time.time() >= 3.0){
                deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                flipper.setPosition(0.5)
            }
            if(time.time() >= 3.5){
                flipper.setPosition(0.7)
            }

            if(time.time() >= 3.5){
                newState(flip_state.STATE_IDLE)
            }
        }

        write()
    }

    fun operate(sequence : Int){
        if(sequence == 0){
            newState(flip_state.STATE_DEPOSIT)
        }
        if(sequence == 1 && prev_sequence != 1){
            newState(flip_state.STATE_DROP)
        }
        if(sequence == 2){
            newState(flip_state.STATE_IDLE)
        }

        if(sequence == 3){
            if(getCase() == 0) {
                //Case regular
                turnPos = case_center_turn_value
                newState(flip_state.STATE_FLIP)
            }
            if(getCase() == 2){
                //Case Right
                turnPos = case_right_turn_value
                newState(flip_state.STATE_REALLIGN)
            }
            if(getCase() == 1){
                //Case Left
                turnPos = case_left_turn_value
                newState(flip_state.STATE_REALLIGN)
            }
        }

        if(sequence == 4 && prev_sequence != 4){
            turnPos = case_center_turn_value
            newState(flip_state.STATE_FLIP)
        }

        prev_sequence = sequence

        if (betterFlipState == flip_state.STATE_FLIP){
            unclamp()
            if(time.time() >= 0.3){ //Wait for setup procedure before flipping
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
                if(time.time() >= 1.0){
                    newState(flip_state.STATE_CLAMP)
                }
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp()
            t.addData("Clamping: ", clamp.getPosition())
            if (time.time() >= 1.0) {   //Wait for 1 second for the flipper to go back
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_IDLE){
            turnPos = turnPos_IDOL
            flipper.setPosition(flipperPos_IDOL)
            turn.setPosition(turnPos)
            deposit.setPosition(DepositPos_IDOL)
            clamp()
        }else if(betterFlipState == flip_state.STATE_DEPOSIT){
            if(time.time() <= 2.0){
                deposit.setPosition(DepositPos)
            }
        }else if(betterFlipState == flip_state.STATE_DROP) {
            unclamp()
            if(time.time() >= 0.3){
                t.addData("Clearing Stone: ", Deposit_Clearance_DROPPING_Block);
                deposit.setPosition(Deposit_Clearance_DROPPING_Block)
            }
        }else if(betterFlipState == flip_state.STATE_REALLIGN){
            unclamp()
            if(time.time() >= 0.5){ //0.5
                flipper.setPosition(Flipper_Midway_REALLIGN)
            }
            if(time.time() >= 1.0){ //0.6
                turn.setPosition(((turnPos-0.5)/2) + 0.5)
                //flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 1.8){
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 2.5){
                clamp()
            }
            if(time.time() >= 3.0){
                deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                flipper.setPosition(0.5)
            }
            if(time.time() >= 3.5){
                flipper.setPosition(0.7)
            }

            if(time.time() >= 3.5){
                newState(flip_state.STATE_IDLE)
            }
        }

        t.addData("time", time.time())
        write()
    }

    fun ShowPos(){
        t.addData("deposit position", deposit.getPosition())
        t.addData("flipper position", flipper.getPosition())
        t.addData("clamp position", clamp.getPosition())
        t.addData("turn position", turn.getPosition())
    }

}