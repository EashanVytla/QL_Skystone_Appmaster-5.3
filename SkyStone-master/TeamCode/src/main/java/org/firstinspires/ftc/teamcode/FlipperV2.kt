package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.State
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class FlipperV2(h : HardwareMap, telemetry : Telemetry){
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
    //var knocker = false
    var telemetry = telemetry

    var previous = false
    var previous2 = false
    var previous3 = false
    var previous4 = false
    var previous5 = false
    var previouscap = false
    var previousclamp = false
    //var prevknocker = false

    val capClamp : Caching_Servo
    //val capDeposit : Caching_Servo

    var clamped = false
    var prev_sequence = -1
    var lock = false
    var first = false

    var previousA = false

    companion object {
        var capped = false
        var rcase = 0
        const val case_right_turn_value = 0.88
        const val case_left_turn_value = 0.15//.1425 //0.1375//0.125
        const val case_center_turn_value = 0.52

        const val handshake_flip_position = 0.4 //THIS IS GOING BACKWARDS 1 -> 0

        const val turnPos_IDOL = 0.48 //0.4975 //0.4935
        const val flipperPos_IDOL = 0.975 //THIS IS GOING BACKWARDS 1 -> 0
        const val flippersafety_IDOL = 0.875
        const val DepositPos_IDOL = 0.1//THIS IS GOING BACKWARDS 1 -> 0

        const val DepositPos = 0.975
        const val Deposit_Clearance_DROPPING_Block = 0.885
        const val Deposit_Clearance_DROPPING_Block_CAP = 0.915
        const val Deposit_Clearance_HANDSHAKE = 0.17
        var knocker = false
    }

    fun flipsafety(){
        flipper.setPosition(flippersafety_IDOL)
    }

    fun getKnocker() : Boolean{
        return knocker
    }

    fun setKnocker(boolean: Boolean){
        knocker = boolean
    }

    var Flipper_Midway_REALLIGN = 0.0 //THIS IS GOING BACKWARDS 1 -> 0

    fun clamp(){
        clamp.setPosition(0.8)  //1.0 with the regular rev servo
    }

    fun unclamp(){
        clamp.setPosition(0.525)  //0.725 with the regular rev servo
    }

    enum class flip_state{
        STATE_CLAMP,
        STATE_FLIP,
        STATE_DEPOSIT,
        STATE_REALLIGN,
        STATE_DROP,
        STATE_IDLE,
        STATE_CLEAR
    }

    var betterFlipState = flip_state.STATE_IDLE

    fun getFlipState() : flip_state{
        return betterFlipState
    }

    fun flipDown(){
        flipper.setPosition(flipperPos_IDOL)
        turn.setPosition(turnPos_IDOL)
    }

    init{
        flipper = Caching_Servo(h, "AlignerTest")
        clamp = Caching_Servo(h,"clamp")
        deposit = Caching_Servo(h,"Deposit")
        turn = Caching_Servo(h,"turn")
        rightpm = Caching_Servo(h, "rightpm")
        leftpm = Caching_Servo(h, "leftpm")
        capClamp = Caching_Servo(h, "cclamp")
        //capDeposit = Caching_Servo(h, "cdeposit")
        sensorDistance = h.get(DistanceSensor::class.java, "cds")
        capClamp.setPosition(0.4)
    }

    fun write(){
        flipper.write()
        clamp.write()
        deposit.write()
        turn.write()
        leftpm.write()
        rightpm.write()
        capClamp.write()
        //capDeposit.write()
    }

    fun start(){
        turn.setPosition(turnPos_IDOL)
        unclamp()
        deposit.setPosition(DepositPos_IDOL)
        time.startTime()
        flipper.setPosition(flipperPos_IDOL)
        //capClamp.setPosition(1.0)
        newState(flip_state.STATE_IDLE)
        write()
    }

    fun startAuto(){
        turn.setPosition(turnPos_IDOL)
        deposit.setPosition(DepositPos_IDOL)
        time.startTime()
        flipper.setPosition(flipperPos_IDOL)
        //capClamp.setPosition(1.0)
        newState(flip_state.STATE_IDLE)
        write()
    }

    fun initialize(){
        clamp()
        //deposit.setPosition(0.0)
        turn.setPosition(turnPos_IDOL)
        leftpm.setPosition(0.3)
        rightpm.setPosition(0.75)
        //capClamp.setPosition(1.0)
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
        first = true
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
            //rcase = 1
            rcase = 0
        } else if (dist >= 9.5 && dist <= 12.0) {
            //Case right
            //9.5 - 10.5
            //rcase = 2
            rcase = 0
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

    fun flipflipper(){
        flipper.setPosition(handshake_flip_position)
        turn.setPosition(case_center_turn_value)
    }

    fun grabPlatform(){
        leftpm.setPosition(0.35)
        rightpm.setPosition(0.65)

        grabbed = true

        leftpm.write()
        rightpm.write()
    }

    fun isGrabbed() : Boolean{
        return grabbed
    }

    fun startKnocker(){
        leftpm.setPosition(0.52)
        rightpm.setPosition(0.425)

        leftpm.write()
        rightpm.write()
    }

    fun partialDeposit(){
        deposit.setPosition(0.9)
    }

    fun resetPlatform(){
        leftpm.setPosition(1.0)
        rightpm.setPosition(0.0)
        grabbed = false
        knocker = false
        leftpm.write()
        rightpm.write()
    }

    private fun isPress(clicked : Boolean, previous : Boolean) : Boolean{
        return clicked && !previous
    }


    fun clampcap(){
        capClamp.setPosition(1.0)
        write()
    }

    fun clampClearStack(){
        clamp.setPosition(0.4)
    }

    var previous10 = false
    var previous11 = false

    fun operate(g1: Gamepad, g2 : Gamepad){
        //if(isPress(g2.b, previous2) /*|| isPress(g1.right_bumper, previous3)*/){
            //newState(flip_state.STATE_IDLE)
        //}
        if(isPress(g1.a, previous)){
            if(grabbed){
                resetPlatform()
                grabbed = false
            }else{
                grabPlatform()
                grabbed = true
            }
        }

        if(isPress(g1.b, previous10)){
            if(knocker){
                resetPlatform()
            }else{
                startKnocker()
            }
            setKnocker(!knocker)
        }

        previous10 = g1.b

        if(isPress(g2.right_bumper, previous4)){
            newState(flip_state.STATE_DEPOSIT)
        }

        if(isPress(g2.dpad_right, previousclamp)){
            if(!clamped){
                capClamp.setPosition(1.0)
                clamped = true
            }else{
                capClamp.setPosition(0.4)
                clamped = false
            }
        }

        if (isPress(g1.right_bumper, previous3) && !Vertical_Elevator.depositCheck){
            //newState(flip_state.STATE_DROP) YES KEEP THIS COMMENTED THIS IS VERY CRUCIAL!!!
        }

        /*
        if(isPress(g2.dpad_left, previouscap)){
            if(!capped){
                capDeposit.setPosition(1.0)
                capped = true
            }else{
                capDeposit.setPosition(0.0)
                capped = false
            }
        }

         */

        if (isPress(g2.a, previousA)){
            newState(flip_state.STATE_IDLE)
        }

        previousclamp = g2.dpad_right
        //previouscap = g2.dpad_left
        previous = g1.a
        previous2 = g2.b
        previous3 = g1.right_bumper
        previous4 = g2.right_bumper
        previousA = g2.a

        if(isPress(g2.left_bumper, previous11)){
            if(Vertical_Elevator.stack_countStatic >= 2){
                if(!Vertical_Elevator.reset){
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
            }else{
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

        }
        previous11 = g2.left_bumper
        if (betterFlipState == flip_state.STATE_FLIP){
            unclamp()
            if(time.time() >= 0.2){ //Wait for setup procedure before flipping
                if(time.time() >= 0.3){
                    turn.setPosition(case_center_turn_value)
                }
                flipper.setPosition(handshake_flip_position)
                if(time.time() >= 1.0){
                    newState(flip_state.STATE_CLAMP)
                }
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp()
            if(time.time() >= 0.4){
                clamp()
                //deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                turnPos = turnPos_IDOL
                flipper.setPosition(flipperPos_IDOL)
                turn.setPosition(turnPos)
            }
            if(time.time() >= 1.0){
                newState(flip_state.STATE_IDLE)
            }
            telemetry.addData("Clearing Handshake: ", Deposit_Clearance_HANDSHAKE)
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
            //unclamp()
            clamp.setPosition(0.5)
            if(time.time() >= 0.3){
                deposit.setPosition(Deposit_Clearance_DROPPING_Block)
            }
        }else if(betterFlipState == flip_state.STATE_REALLIGN){

            if(time.time() >= 0.25 && time.time() <= 0.55){ //0.5
                flipper.setPosition(Flipper_Midway_REALLIGN)
            }
            if(time.time() >= 0.55 && time.time() <= 0.9){ //0.6
                turn.setPosition(((turnPos-0.5)/2) + 0.5)
                //turn.setPosition(turnPos)
                //flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 0.9 && time.time() <= 1.25){
                turn.setPosition(turnPos)
                if(time.time() >= 1.1){
                    flipper.setPosition(handshake_flip_position)
                }
            }
            if(time.time() >= 1.5){
                clamp()
            }else{
                unclamp()
            }
            if(time.time() >= 1.85 && time.time() <= 2.25){
                deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                flipper.setPosition(0.5)
            }
            if(time.time() >= 2.25){
                flipper.setPosition(0.7)
            }

            if(time.time() >= 2.25){
                newState(flip_state.STATE_IDLE)
            }
        }
        write()
    }

    fun operate(sequence : Int){
        if(sequence == 0 && prev_sequence != 0){
            newState(flip_state.STATE_DEPOSIT)
        }
        if(sequence == 1 && prev_sequence != 1){
            newState(flip_state.STATE_DROP)
        }
        if(sequence == 2 && prev_sequence != 2){
            newState(flip_state.STATE_IDLE)
        }

        if(sequence == 3 && prev_sequence != 3){
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

        if(sequence == 4 && prev_sequence != 4){
            turnPos = case_center_turn_value
            newState(flip_state.STATE_FLIP)
        }

        //if(sequence == 5 && prev_sequence != 5){
          //  newState(flip_state.STATE_DROP)
        //}

        prev_sequence = sequence

        if (betterFlipState == flip_state.STATE_FLIP){
            unclamp()
            if(time.time() >= 0.2){ //Wait for setup procedure before flipping
                if(time.time() >= 0.3){
                    turn.setPosition(case_center_turn_value)
                }
                flipper.setPosition(handshake_flip_position)
                if(time.time() >= 1.0){
                    newState(flip_state.STATE_CLAMP)
                }
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp()
            if(time.time() >= 0.4){
                clamp()
                //deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                turnPos = turnPos_IDOL
                flipper.setPosition(flipperPos_IDOL)
                turn.setPosition(turnPos)
            }
            if(time.time() >= 1.0){
                newState(flip_state.STATE_IDLE)
            }
            telemetry.addData("Clearing Handshake: ", Deposit_Clearance_HANDSHAKE)
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
            //unclamp()
            if(clamped){
                clamp.setPosition(0.5)
                if(time.time() >= 0.3){
                    deposit.setPosition(Deposit_Clearance_DROPPING_Block)
                }
            }else{
                clamp.setPosition(0.5)
                if(time.time() >= 0.3){
                    deposit.setPosition(Deposit_Clearance_DROPPING_Block_CAP)
                }
            }

        }else if(betterFlipState == flip_state.STATE_REALLIGN){
            unclamp()
            if(time.time() >= 0.1){
                flipper.setPosition(handshake_flip_position)
                //turn.setPosition(turnPos)
            }
            if (time.time() >= 0.2){
                turn.setPosition(turnPos)
            }
            if(time.time() >= 0.8){
                clamp()
            }
            if(time.time() >= 1.1){
                deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                turnPos = turnPos_IDOL
                flipper.setPosition(flipperPos_IDOL)
                turn.setPosition(turnPos)
                clamp()
            }
            if(time.time() >= 1.8){
                newState(flip_state.STATE_IDLE)
            }
        }
        write()
    }

    fun ShowPos(){
        t.addData("deposit position", deposit.getPosition())
        t.addData("flipper position", flipper.getPosition())
        t.addData("clamp position", clamp.getPosition())
        t.addData("turn position", turn.getPosition())
    }
}