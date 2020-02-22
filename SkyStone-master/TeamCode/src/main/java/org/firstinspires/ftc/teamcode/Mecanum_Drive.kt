package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.Pure_Pursuit.CurvePoint
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer
import org.firstinspires.ftc.teamcode.Pure_Pursuit.MathFunctions
import org.firstinspires.ftc.teamcode.Pure_Pursuit.RobotMovement
import org.firstinspires.ftc.teamcode.Universal.Math.Pose
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.*

class Mecanum_Drive(hardwareMap : HardwareMap, telemetry: Telemetry){
    var motors = ArrayList<Caching_Motor>()
    var theta = 0.0

    var prev_pos = ArrayList<Double>()
    var prev_heading = 0.0

    val imu : LynxEmbeddedIMU
    val hub : ExpansionHubEx
    val hub2 : ExpansionHubEx

    var currentWriteIndex = 0

    val TRACK_WIDTH = 15.75
    val DRIVE_WIDTH = 13.625

    var pos = Pose()

    val EPSILON = 0.001

    var currHeading = 0.0
    var headingReadCount = 0
    var headingAccessCount = 0
    val headingUpdateFrequency = 1.0

    var rangle : Double = 0.0
    //var angle: Double = 0.toDouble()
    var prevheading = 0.0
    var slowmode2 = false

    var integralError = 0.0
    var prev_time = System.currentTimeMillis()
    var dt = 0.0

    var time = ElapsedTime()

    var scale = 1.0
    var slow_mode3 = false

    var orientation : Orientation
    var fine_tune = 1.0

    var previous = false
    var previous2 = false
    var previous3 = false
    var previous4 = false
    var previous5 = false
    var slow_mode4 = false
    var slow_mode5 = false
    var slow_mode = false
    var previous6 = false
    var previous7 = false
    var previous8 = false
    var previous9 = false
    var previous10 = false

    //var pid: PID

    var mode = false

    var headingerror = 0.0
    //var headingerror: Double = 0.toDouble()

    var headingLock = false

    //Turn Coefficients
    val kpA = 0.82 //0.39
    val kiA = 0.0
    val kdA = 0.07 //0.6

    //Straight Line Coefficients
    val kpstr = 0.05     //0.03
    val kistr = 0.0      //0.0
    val kdstr = 0.0125      //9.25

    //Strafe PID coeffients
    val kpstf = 0.13      //0.09
    val kistf = 0.002
    val kdstf = 0.02      //10.0

    //feedrforward
    val kv = 0.02
    val ka = 0.005
    val kstatic = 0.0

    var preverror = 0.0
    var preverrorstr = 0.0
    var preverrorstf = 0.0
    var integralEstraight = 0.0
    var integralEdrift = 0.0
    var integralEstrafe = 0.0
    var rotintegralError = 0.0
    var rotprevheading = 0.0
    var vectorAngle = 0.0

    var pidstr : PIDFController
    var pidstf : PIDFController
    var pidr : PIDFController
    var telemetry = telemetry

    var position = Pose2d(0.0, 0.0, 0.0)

    var p = Pose2d(0.0, 0.0, 0.0)
    var prevpos = DoubleArray(4)
    var robotHeading = 0.0
    //var Deposit : Servo



    var fine_tune_rot = 0.0

    enum class aastate{
        STATE_IDOL,
        STATE_MOVING,
    }

    var AutoAllignSt = aastate.STATE_IDOL

    enum class State{
        STATE_STRAFE,
        STATE_IDLE
    }

    var state = State.STATE_IDLE

    var mStateTime = ElapsedTime()
    var angleOffset = 0.0

    companion object{
        var refresh_rate = 0.5  //ngl this is kinda scary but you do what u gotta do to get 300 hz

        const val kpA = 0.39//0.35625
        const val kiA = 0.008//0.005
        const val kdA = 0.2
        const val kp = 1.4
        var capstone = false
        var capTime = ElapsedTime()
        var currentPos = Pose2d(0.0,0.0,0.0)

        var automateLock2 = false
    }
    var automateLock = false

    fun setCurrentPos(pos : Pose2d){
        currentPos = pos
    }

    fun getCurrentPos() : Pose2d{
        return currentPos
    }

    var data : RevBulkData
    var data2 : RevBulkData
    var flip : FlipperV2

    var odos : ThreeWheelTrackingLocalizer
    init{
        motors.add(Caching_Motor(hardwareMap, "up_left"))
        motors.add(Caching_Motor(hardwareMap, "back_left"))
        motors.add(Caching_Motor(hardwareMap, "back_right"))
        motors.add(Caching_Motor(hardwareMap, "up_right"))
        odos = ThreeWheelTrackingLocalizer(hardwareMap)
        flip = FlipperV2(hardwareMap, telemetry)
        //pid = PID(hardwareMap, telemetry)

        pidstr = PIDFController(PIDCoefficients(kpstr, kistr, kdstr), kv, ka, kstatic)
        pidstf = PIDFController(PIDCoefficients(kpstf, kistf, kdstf), kv, ka, kstatic)
        pidr = PIDFController(PIDCoefficients(kpA, kiA, kdA))

        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        hub2 = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 1")
        //Deposit = hardwareMap.get(Servo::class.java, "")

        data = hub.bulkInputData
        data2 = hub2.bulkInputData

        /*
        leftWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237) //1.5144 0.0361262
        rightWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237) //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642 * 2 * 0.797, 0.0) //1.50608 -0.221642

         */

        /*
        leftWheel.setBehavior(1.5385, -0.319237) //1.5144 0.0361262

        rightWheel.setBehavior(1.5385, -0.319237) //1.5204 -0.00305571

        strafeWheel.setBehavior(1.53642, 0.0) //1.50608 -0.221642

         */


        motors.forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors[0].motor.direction = DcMotorSimple.Direction.REVERSE
        motors[1].motor.direction = DcMotorSimple.Direction.REVERSE


        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        orientation = imu.angularOrientation
        time.startTime()
        capTime.startTime()

        RobotMovement.setTelemetry(telemetry)
    }

    fun newState(s : State){
        mStateTime.reset()
        state = s
    }

    private fun tweakRefreshRate(gamepad : Gamepad){
        if (gamepad.dpad_up){
            raiseRefreshRate()
        }
        else if (gamepad.dpad_down){
            dropRefreshRate()
        }
    }

    private fun raiseRefreshRate(){
        var num = (1 / refresh_rate).toInt()
        num++
        refresh_rate = 1.div(num.toDouble())
    }

    private fun dropRefreshRate(){
        var num = (1 / refresh_rate).toInt()
        if (num > 1) {
            num--
            refresh_rate = 1.div(num.toDouble())
        }
    }

    fun getRefreshRate() : Double{
        return refresh_rate
    }

    fun getPower() : Array<Double>{
        return arrayOf(motors[0].prev_power, motors[1].prev_power, motors[2].prev_power, motors[3].prev_power)
    }

    fun getOverallPower() : Double{
        return (motors[0].motor.power + motors[1].motor.power + motors[2].motor.power + motors[3].motor.power)/4
    }

    fun pivotTo(heading : Double, fPower : Double) : Boolean{
        val error = heading - getExternalHeading()
        val power = kpA * error * 0.5
        if (abs(error) > Math.toRadians(10.0)) {
            setPower(fPower, 0.0, power)
            return true
        }
        else{
            setPower(0.0, 0.0, 0.0)
            return false
        }
    }

    fun scalePower(speed: Double) : Double{
        //return 0.5 * Math.pow(2 * (speed - 0.5), 3.0) + 0.5
        /*
        if(speed != 0.0){
            return Math.pow(speed, 3.0)/Math.abs(speed)
        }else {
            return 0.0
        }
         */
        return speed
    }

    /*
    fun autoAllign(state : AutoAllignSt, gamepad: Gamepad){
        if(state == AutoAllignSt.STATE_IDLE){
            setPower(fine_tune * scalePower(gamepad.left_stick_y.toDouble()), fine_tune * scalePower(gamepad.left_stick_x.toDouble()), -0.5 * gamepad.right_stick_x.toDouble())
        }else if(state == AutoAllignSt.STATE_STRAFE){
            setPower(0.0, gamepad.left_stick_y.toDouble(), 0.0)
        }else if(state == AutoAllignSt.STATE_TURN){
            if(time.time() >= 2.0){
                targetTurn((Math.PI/2 - angleOffset))
            }
        }
    }

     */

    fun isPress2(value : Boolean, previous : Boolean) : Boolean{
        return value && !previous
    }

    fun drive(gamepad : Gamepad, gamepad2: Gamepad){
        capstone = false
        val data = hub2.bulkInputData
        slowmode2 = gamepad.right_trigger > 0.0
        odos.update()
        odos.dataUpdate(data)
        var heading = 0.0

        if(isPress2(gamepad2.x, previous2)){
            slow_mode = true
        }
        /*
        if(isPress2(gamepad.b, previous10)){
            if(flip.getKnocker()){
                flip.resetPlatform()
            }else{
                flip.startKnocker()
            }
            flip.setKnocker(!flip.getKnocker())
        }

         */

        if (isPress2(gamepad2.b, previous6) || isPress2(gamepad.right_bumper, previous7)) {
            slow_mode = false
        }

        if (isPress2(gamepad.left_bumper, previous3)){
            //automateLock = !automateLock
            slowmode2 = false
            slow_mode = false
            slow_mode3 = false
            slow_mode4 = false
            slow_mode5 = false
        }

        if(isPress2(gamepad2.dpad_left, previous8)){
            //capstone = !capstone
            slow_mode4 = !slow_mode4
        }

        if (!isInBounds(gamepad)){
            automateLock = false
        }

        if(isPress2(gamepad.a, previous5)){
            slow_mode3 = !slow_mode3
            slow_mode = false
            slowmode2 = false
            slow_mode4 = false
        }

        previous2 = gamepad2.x
        previous3 = gamepad.left_bumper
        previous4 = gamepad.y
        previous5 = gamepad.a
        previous6 = gamepad2.b
        previous7 = gamepad.right_bumper
        previous8 = gamepad2.dpad_left
        previous10 = gamepad.b

        if (slow_mode || slowmode2 || slow_mode4){ //Slides and everything else
            fine_tune = 0.4
            fine_tune_rot = 0.3
        }else if(slow_mode3){ //Foundation
            fine_tune = 0.45
            fine_tune_rot = 0.3
        } else{
            fine_tune = 1.0
            fine_tune_rot = 0.5
        }

        if(Math.hypot(gamepad.right_stick_x.toDouble(), gamepad.right_stick_y.toDouble()) >= 0.15 || Math.hypot(gamepad.left_stick_x.toDouble(), gamepad.left_stick_y.toDouble()) >= 0.15){
            automateLock = false
            automateLock2 = false
        }

        if (!automateLock && !automateLock2) {
            setPower(fine_tune * scalePower(gamepad.left_stick_y.toDouble()), fine_tune * scalePower(gamepad.left_stick_x.toDouble()), -fine_tune_rot * gamepad.right_stick_x.toDouble())
        }else if(automateLock2){
            if(capTime.time() >= 1.0){
                goToDeposit(1.0, 1.0, 1.0)
            }
            Flipper.capped = true
        }else if(automateLock){
            goToDeposit(1.0, 1.0, 1.0)
            //capstoneStrafe()
            //targetTurn(Vector2d(fine_tune * gamepad.left_stick_y.toDouble(), fine_tune * gamepad.left_stick_x), Math.PI / 2)
        }
        //telemetry.addData("Capstone Readiness: ", capstone)
        write()
    }

    fun followPath(pathPoints : ArrayList<CurvePoint>, followAngle : Double, p : Pose2d){
        var lookahead : Pose2d = RobotMovement.followPath(pathPoints, followAngle, p)
        telemetry.addData("Lookahead Point: ", lookahead)
        telemetry.addData("Error: ", p.vec().distTo(lookahead.vec()))
        goToPoint(p, lookahead, pathPoints.get(RobotMovement.getCurrentLine(MathFunctions.poseToPoint(lookahead), pathPoints)).moveSpeed, pathPoints.get(RobotMovement.getCurrentLine(MathFunctions.poseToPoint(lookahead), pathPoints)).moveSpeed, pathPoints.get(RobotMovement.getCurrentLine(MathFunctions.poseToPoint(lookahead), pathPoints)).turnSpeed)
    }

    fun followPath(pathPoints : ArrayList<CurvePoint>, followAngle : Double, p : Pose2d, power : Double, holdAngle : Double = 0.0){
        var lookahead = RobotMovement.followPath(pathPoints, followAngle, holdAngle, p)
        telemetry.addData("Lookahead Point: ", lookahead)
        goToPoint(p, lookahead, power, power, power)
    }

    fun goToPoint(p : Pose2d, target : Pose2d, strspeed: Double, stfspeed : Double, rspeed : Double){
        pidstr.setOutputBounds(-strspeed, strspeed)
        pidstf.setOutputBounds(-stfspeed, stfspeed)
        pidr.setOutputBounds(-rspeed, rspeed)
        var heading = p.heading
        var target_heading = target.heading
        var turnPower = 0.0

        if(p.heading <= Math.PI){
            heading = p.heading
        }else{
            heading = -((2 * Math.PI ) - p.heading)
        }

        if(Math.abs(target.heading - heading) >= Math.toRadians(180.0)){
            target_heading = -((2 * Math.PI) - target.heading)
        }

        telemetry.addData("heading: ", heading)

        pidr.targetPosition = target_heading
        pidstr.targetPosition = target.y
        pidstf.targetPosition = target.x

        turnPower = pidr.update(heading)

        centricSetPower(-pidstf.update(p.x), -pidstr.update(p.y), turnPower, p.heading)
        telemetry.addData("Error: ", target - p)
    }

    fun isInBounds(gamepad : Gamepad) : Boolean{
        return gamepad.left_stick_x < 0.3 && gamepad.left_stick_y < 0.3 && gamepad.right_stick_x < 0.3 && gamepad.right_stick_y < 0.3
    }

    fun goToDeposit(strspeed : Double, stfspeed : Double, rspeed : Double){
        pidstr.setOutputBounds(-strspeed, strspeed)
        pidstf.setOutputBounds(-stfspeed, stfspeed)
        pidr.setOutputBounds(-rspeed, rspeed)

        var heading : Double

        if(currentPos.heading <= Math.PI){
            heading = currentPos.heading
        }else{
            heading = -((2 * Math.PI ) - currentPos.heading)
        }
        telemetry.addData("heading: ", heading)

        pidr.targetPosition = 0.0
        pidstr.targetPosition = 0.0
        pidstf.targetPosition = 0.0

        //val state = strprofile[time]
        val powerstr = pidstr.update(currentPos.x/*, state.v, state.a*/)
        telemetry.addData("current pos: ", currentPos.toString())
        telemetry.addData("Capstone Readiness? ", capstone)

        centricSetPower(-powerstr, pidstf.update(-currentPos.y), pidr.update(heading), heading)
    }





    /*
    fun capstoneStrafe(){
        if (state == State.STATE_STRAFE){
            automateLock = true
            if (mStateTime.time() >= 1.5){
                automateLock = false
                newState(State.STATE_IDLE)
            }
            else if (localizer.getForwardDist() > 3.0){ //todo: change this to strafe val
                automateLock = false
                newState(State.STATE_IDLE)
            }
            else{
                setPower(0.0, 0.3, 0.0)
            }
        }
    }

     */

    fun toggleHeadingLock(){
        headingLock = !headingLock
        integralError = 0.0
    }

    fun isPress(test : Boolean) : Boolean{
        return test && !previous
    }

    fun angleWrap(angle : Double) : Double{
        return (angle + (2 * Math.PI)) % (2 * Math.PI)
    }

    fun f_drive(gamepad1 : Gamepad){
        //tweakRefreshRate(gamepad1)
        if (isPress(gamepad1.right_bumper)){
            slow_mode = !slow_mode
        }
        previous = gamepad1.right_bumper

        if (slow_mode){
            fine_tune = 0.5
        }
        else{
            fine_tune = 1.0
        }

        val r = hypot(gamepad1.left_stick_y, gamepad1.left_stick_x)
        val theta = atan2(/*-*/gamepad1.left_stick_y, gamepad1.left_stick_x).toDouble()
        val v = Vector2(r * cos(theta), r * sin(theta))
        v.rotate(angleWrap(getExternalHeading()))
        setPower(v, -0.5 * gamepad1.right_stick_x)
        write()
    }

    fun centricSetPower(y : Double, x : Double, rot : Double, heading: Double){
        val r = hypot(y, x)
        val theta = atan2(/*-*/y, x).toDouble()
        val v = Vector2(r * cos(theta), r * sin(theta))
        v.rotate(heading)
        setPower(v, rot)
        write()
    }

    fun read(data : RevBulkData) {
        motors.forEach {
            it.read(data)
        }

        headingReadCount++
        if (headingAccessCount.toDouble() / headingReadCount.toDouble() < headingUpdateFrequency || true) {
            headingAccessCount++
            currHeading = imu.angularOrientation.firstAngle.toDouble()
            orientation = imu.angularOrientation
        }
        headingReadCount++
    }

    fun setPower(y : Double, x : Double, rightX : Double){
        var FrontLeftVal = y - x + rightX
        var FrontRightVal = y + x - rightX
        var BackLeftVal = y + x + rightX
        var BackRightVal = y - x - rightX

        //Move range to between 0 and +1, if not already
        val wheelPowers = doubleArrayOf(FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal)
        Arrays.sort(wheelPowers)
        if (wheelPowers[3] > 1) {
            FrontLeftVal /= wheelPowers[3]
            FrontRightVal /= wheelPowers[3]
            BackLeftVal /= wheelPowers[3]
            BackRightVal /= wheelPowers[3]
        }
        motors[0].setPower(FrontLeftVal)
        motors[1].setPower(BackLeftVal)
        motors[2].setPower(BackRightVal)
        motors[3].setPower(FrontRightVal)
    }

    fun setPower(v : Vector2, rightX : Double){
        setPower(v.y, v.x, rightX)
    }

    fun write(){
        motors[currentWriteIndex].write()
        currentWriteIndex = (currentWriteIndex + 1) % 4
    }

    fun getExternalHeading() : Double{
        return orientation.firstAngle.toDouble()
    }

    fun getEstimatedPose(){
        val wheelVelocities = ArrayList<Double>()
        motors.forEachIndexed{index, motor ->
            wheelVelocities.add(motor.getCurrentPosition() - prev_pos[index])
            prev_pos[index] = motor.getCurrentPosition().toDouble()
        }
        val k = (TRACK_WIDTH + DRIVE_WIDTH) / 2.0
        val (frontLeft, rearLeft, rearRight, frontRight) = wheelVelocities

        val heading = getExternalHeading()
        val offset = Pose(wheelVelocities.sum(), rearLeft + frontRight - frontLeft - rearRight, heading - prev_heading)
        prev_heading = heading
        //relativeOdometryUpdate(offset)
    }

    fun getAngle() : Double{
        return angleWrap(getExternalHeading())
    }

    fun setPowerSide(left : Double, right : Double){
        motors[0].setPower(left)
        motors[1].setPower(left)
        motors[2].setPower(right)
        motors[3].setPower(right)
    }
    /*fun relativeOdometryUpdate(robotPoseDelta : Pose){
        val dtheta = robotPoseDelta.angle

        val (sineTerm, cosTerm) = if (abs(dtheta) < EPSILON) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1 - cos(dtheta)) / dtheta
        }

        val fieldPositionDelta = Vector2(sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y)
        fieldPositionDelta.rotate(pos.angle)

        val fieldPoseDelta = Pose(fieldPositionDelta.x, fieldPositionDelta.y, dtheta)

        pos.add(fieldPoseDelta)
    }
    */

    fun targetTurn(targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        rangle = angleWrap(getExternalHeading())
        headingerror = targetAngle - rangle

        if (abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - abs(headingerror))
            }
            else{
                headingerror = ((Math.PI * 2) - abs(headingerror))
            }
        }

        val prop = headingerror * kpA
        val integral = integralError * kiA
        val deriv = ((headingerror - prevheading) * kdA / dt)

        val power = prop + integral + deriv
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(0.0, 0.0, Range.clip(power, -1.0, 1.0))
        //write()
    }


    fun targetTurnPlatform(targetAngle : Double, heading : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        headingerror = targetAngle - heading

        if (abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - abs(headingerror))
            }
            else{
                headingerror = ((Math.PI * 2) - abs(headingerror))
            }
        }

        val prop = headingerror * kp
        val power = prop
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(0.0, 0.0, Range.clip(power, -1.0, 1.0))
    }

    fun targetTurnPlatform(targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        rangle = angleWrap(getExternalHeading())
        headingerror = targetAngle - rangle

        if (abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - abs(headingerror))
            }
            else{
                headingerror = ((Math.PI * 2) - abs(headingerror))
            }
        }

        val prop = headingerror * kp
        val power = prop
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(0.0, 0.0, Range.clip(power, -1.0, 1.0))
    }

    fun targetTurn(drive : Vector2d, targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        rangle = angleWrap(getExternalHeading())
        headingerror = targetAngle - rangle

        val prop = headingerror * kpA
        val integral = integralError * kiA
        val deriv = (headingerror - prevheading) * kdA / dt

        val power = prop + integral + deriv
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(drive.y, drive.x, Range.clip(power, -1.0, 1.0))
    }
}