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
        const val servoRotateOpen = 1.0
        const val servoRotateClose = 0.0

        const val servoSustain1Open = 1.0
        const val servoSustain1Close = 0.0

        const val servoSustain2Open = 1.0
        const val servoSustain2Close = 0.0
    }

    val continuosServo = hwMap.get(CRServo::class.java, "continousServo") ?: throw Exception("Failed to find servo continousServo")
    val sustainServo1 = hwMap.servo["sustainServo1"] ?: throw Exception("Failed to find servo sustainServo1")
    val sustainServo2 = hwMap.servo["sustainServo2"] ?: throw Exception("Failed to find servo sustainServo2")
    val servoRotate = hwMap.servo["servoRotate"] ?: throw Exception("Failed to find servo servoRotate")

    var sustainServo1Position : Double = servoSustain1Close
    var sustainServo2Position : Double = servoSustain2Close
    var rotateServoPosition : Double = servoRotateClose

    init {
        sustainServo1.position = servoSustain1Close
        sustainServo2.position = servoSustain2Close
    }

    fun extendMeasure()
    {
        continuosServo.direction = DcMotorSimple.Direction.REVERSE
        continuosServo.power = 0.7
    }

    fun shrinkMeasure()
    {
        continuosServo.direction = DcMotorSimple.Direction.FORWARD
        continuosServo.power = 0.5
    }

    fun stopMeasure()
    {
        continuosServo.power = 0.0
    }

    fun moveUp()
    {
        sustainServo1Position+=0.03
        sustainServo2Position+=0.03

        if(sustainServo1Position > servoSustain1Open)
            sustainServo1Position = servoSustain1Open


        if(sustainServo2Position > servoSustain2Open)
            sustainServo2Position = servoSustain2Open


        sustainServo1.position = sustainServo1Position
        sustainServo2.position = sustainServo2Position
    }

    fun moveDown()
    {
        sustainServo1Position-=0.03
        sustainServo2Position-=0.03

        if(sustainServo1Position < servoSustain1Close)
            sustainServo1Position = servoSustain1Close

        if(sustainServo2Position < servoSustain2Close)
            sustainServo2Position = servoSustain2Close

        sustainServo1.position = sustainServo1Position
        sustainServo2.position = sustainServo2Position
    }

    fun rotateLeft()
    {
        rotateServoPosition += 0.3

        if(rotateServoPosition > servoRotateOpen)
            rotateServoPosition = servoRotateOpen

        servoRotate.position = rotateServoPosition
    }

    fun rotateRight()
    {
        rotateServoPosition -= 0.3
        if(rotateServoPosition < servoRotateClose)
            rotateServoPosition = servoRotateClose

        servoRotate.position = rotateServoPosition
    }


    fun stop(){
        continuosServo.power = 0.5
    }
}