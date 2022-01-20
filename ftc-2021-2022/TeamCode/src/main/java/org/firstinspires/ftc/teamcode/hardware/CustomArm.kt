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
        const val ARM_DEPLOY = 300
        const val ARM_CLOSED = 0

        const val MULTIPLIER_MOTOR = 10
        const val MULTIPLIER_SERVO = 0.05

        const val servoHelpOpenPickup = 0.0
        const val servoHelpOpenDeploy = 0.0
        const val servoHelpClose = 1.0

        const val servoMagnetOpen = 0.0
        const val servoMagnetClose = 1.0
    }

    val armMotor = hwMap.dcMotor["armMotor"] ?: throw Exception("Failed to find motor armMotor")
    val clawServo = hwMap.servo["clawServo"] ?: throw Exception("Failed to find servo clawServo")
    //val servoHelp = hwMap.servo["ServoHelp"] ?: throw Exception("Failed to find servo servoHelp")

    var armPosition: Int = 0
    var servoHelpPosition: Double = servoHelpClose
    var servoMagnetPosition: Double = servoMagnetClose

    init {
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        armMotor.direction = DcMotorSimple.Direction.FORWARD
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        closeServoM()
        releaseServoM()
    }

    fun openArmPickup(){
        armPosition = ARM_OPENED
        armMotor.targetPosition = armPosition
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armMotor.power = 1.0

        setServoPositions(servoHelpOpenPickup)
    }

    fun openArmDeploy(){
        armPosition = ARM_DEPLOY
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

    fun moveArm(direction: Int){
        if(direction > 0)
        {
            armPosition += MULTIPLIER_MOTOR
            if(armPosition > ARM_OPENED)
                armPosition = ARM_OPENED
            armMotor.targetPosition = armPosition
            armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            armMotor.power = 1.0

            servoHelpPosition -= MULTIPLIER_SERVO
            if(servoHelpPosition < servoHelpClose)
                servoHelpPosition = servoHelpClose
        }
        else
        {
            armPosition -= MULTIPLIER_MOTOR
            if(armPosition < ARM_CLOSED)
                armPosition = ARM_CLOSED
            armMotor.targetPosition = armPosition
            armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            armMotor.power = 1.0

            servoHelpPosition += MULTIPLIER_SERVO
            if(servoHelpPosition > servoHelpOpenPickup)
                servoHelpPosition = servoHelpOpenPickup
        }
    }

    fun stop(){
        armMotor.power = 0.0
    }

    fun releaseServoM(){
        setServoPositions(servoMagnetOpen)
    }

    fun closeServoM(){
        setServoPositions(servoMagnetClose)
    }


    fun closeServoHelp() {
        setServoPositions(servoHelpClose)
    }

    fun setServoPositions(pos: Double){
        clawServo.position = pos
    }
}