package org.firstinspires.ftc.teamcode

import kotlin.math.abs

class PDF_Controller(coeffs : PDF_Coefficients) {
    var coeffs = coeffs

    private var prev_error = 0.0
    private var prev_time = System.currentTimeMillis()

    private val EPSILON = 0.001 //todo: tune me

    var isSteadyState = false

    fun update(targetState : Double, currentState : Double) : Double{ //returns whether component is at steady state
        val FF = coeffs.kF * targetState

        val P = coeffs.kP * (targetState - currentState)
        val D = coeffs.kD * ((targetState - currentState) - prev_error) / (System.currentTimeMillis() - prev_time)
        prev_time = System.currentTimeMillis()
        prev_error = targetState - currentState

        isSteadyState = abs(P + D) < EPSILON
        return P + D + FF
    }
}

class PDF_Coefficients(kP : Double, kD : Double, kF : Double){
    val kP = kP
    val kD = kD
    val kF = kF
}