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
        const val servoOpenParallelOdo = 0.7
        const val servoCloseParallelOdo = 0.0

        const val servoOpenPerpendicularOdo = 0.6
        const val servoClosePerpendicularOdo = 0.27
    }

    val servoParallelOdo = hwMap.servo["parallelOdoServo"] ?: throw Exception("Failed to find servo parallelOdoServo")
    val servoPerpendicularOdo = hwMap.servo["perpendicularOdoServo"] ?: throw Exception("Failed to find perpendicularOdoServo")

    var currentPosParallel = 0.0
    var currentPosPerpendicular = 0.0

    init {
        openServos()
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

    fun move1UpParallel(){
        currentPosParallel += 0.01
        if(currentPosParallel > 1)
            currentPosParallel = 1.0
        servoParallelOdo.position = currentPosParallel
    }

    fun move1UpPerpendicular(){
        currentPosPerpendicular += 0.01
        if(currentPosPerpendicular > 1)
            currentPosPerpendicular = 1.0
        servoPerpendicularOdo.position = currentPosPerpendicular
    }

    fun move1DownParallel(){
        currentPosParallel -= 0.01
        if(currentPosParallel < 0)
            currentPosParallel = 0.0
        servoParallelOdo.position = currentPosParallel
    }

    fun move1DownPerpendicular(){
        currentPosPerpendicular -= 0.01
        if(currentPosPerpendicular < 0)
            currentPosPerpendicular = 0.0
        servoPerpendicularOdo.position = currentPosPerpendicular
    }

    fun stop(){

    }
}