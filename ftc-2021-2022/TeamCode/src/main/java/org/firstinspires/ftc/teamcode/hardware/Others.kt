package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.Exception

/**
 * Others
 *
 * This class controls the less important hardware such as odometry servos
 */

class Others(hwMap: HardwareMap) {
    companion object {
        const val servoOpenParallelOdo = 1.0
        const val servoCloseParallelOdo = 0.0

        const val servoOpenPerpendicularOdo = 1.0
        const val servoClosePerpendicularOdo = 0.0
    }

    val servoParallelOdo = hwMap.servo["parallelOdoServo"] ?: throw Exception("Failed to find servo parallelOdoServo")
    val servoPerpendicularOdo = hwMap.servo["perpendicularOdoServo"] ?: throw Exception("Failed to find perpendicularOdoServo")

    init {
        closeServos()
    }

    fun closeServos() {
        closeParallServo()
        closePerpendicularServo()
    }

    fun openServos() {
        openParallelServo()
        openPerpendicularServo()
    }

    fun openParallelServo() {
        setParallelServoPositions(servoOpenParallelOdo)
    }

    fun openPerpendicularServo() {
        setPerpendicularServoPositions(servoOpenPerpendicularOdo)
    }

    fun closeParallServo() {
        setParallelServoPositions(servoCloseParallelOdo)
    }

    fun closePerpendicularServo() {
        setPerpendicularServoPositions(servoClosePerpendicularOdo)
    }

    fun setParallelServoPositions(pos: Double) {
        servoParallelOdo.position = pos
    }

    fun setPerpendicularServoPositions(pos: Double){
        servoPerpendicularOdo.position = pos
    }

    fun stop(){

    }
}