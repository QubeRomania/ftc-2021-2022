package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.Exception

/**
 * Custom Arm
 *
 * This class controls the arm which picks up the custom element
 */

class CustomArm(hwMap: HardwareMap) {
    companion object {
        const val ARM_OPENED = 500
        const val ARM_CLOSED = 0

        const val MULTIPLIER = 10


        const val servoClawOpen = 0.0
        const val servoClawClose = 1.0
    }

    val armMotor = hwMap.dcMotor["armMotor"] ?: throw Exception("Failed to find motor armMotor")
    val clawServo = hwMap.servo["clawServo"] ?: throw Exception("Failed to find servo clawServo")

    var armPosition: Int = 0

    init {
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        armMotor.direction = DcMotorSimple.Direction.FORWARD
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        closeServo()
    }

    fun openArm(){
        armPosition = ARM_OPENED
        armMotor.targetPosition = armPosition
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armMotor.power = 1.0
    }

    fun closeArm(){
        armPosition = ARM_CLOSED
        armMotor.targetPosition = armPosition
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armMotor.power = 1.0
    }

    fun moveArm(power: Double){
        armPosition += (power* MULTIPLIER).toInt()

        if(armPosition > ARM_OPENED)
            armPosition = ARM_OPENED
        else if(armPosition < ARM_CLOSED)
            armPosition = ARM_CLOSED

        armMotor.targetPosition = armPosition
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armMotor.power = 1.0
    }

    fun stop(){
        armMotor.power = 0.0
    }

    fun releaseServo(){
        setServoPositions(servoClawOpen)
    }

    fun closeServo(){
        setServoPositions(servoClawClose)
    }

    fun setServoPositions(pos: Double){
        clawServo.position = pos
    }
}