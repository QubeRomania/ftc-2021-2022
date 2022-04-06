package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.rev.RevTouchSensor
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
        const val servoYOpen = 1.0
        const val servoYClose = 0.0

        const val servoZOpen = 1.0
        const val servoZClose = 0.3

        const val servoYInit = 0.73
        const val servoZInit = 0.31

        const val servoYOut = 0.69
        const val servoZOut = 0.54

        const val servoYIn = 0.75
        const val servoZIn = 0.41

        const val servoYD = 0.6
        const val servoZD = 0.45

        const val modifier = 0.005

    }

    var currentPosition = 0.0

    val servoY = hwMap.servo["servoY"] ?: throw Exception("Failed to find servoY")
    val servoZ = hwMap.servo["servoZ"] ?: throw Exception("Failed to find servoZ")
    val continousServo = hwMap.get(CRServo::class.java,"continousServo") ?: throw Exception("Failed to find CRServo continousServo")

    var servoYPosition : Double = servoYClose
    var servoZPosition : Double = servoZClose

    init {
        continousServo.direction = DcMotorSimple.Direction.REVERSE
        servoY.position = servoYInit
        servoYPosition = servoYInit
        servoZPosition = servoYInit
        servoZ.position = servoZInit
    }

    fun extendMeasure()
    {
        continousServo.power = -1.0
    }

    fun shrinkMeasure()
    {
        continousServo.power = 1.0
    }

    fun stopMeasure()
    {
        continousServo.power = 0.0
    }

    fun moveUp()
    {
        servoYPosition += modifier

        if(servoYPosition > servoYOpen)
            servoYPosition = servoYOpen

        servoY.position = servoYPosition
    }

    fun moveDown()
    {
        servoYPosition -= modifier

        if(servoYPosition < servoYClose)
            servoYPosition = servoYClose

        servoY.position = servoYPosition
    }

    fun rotateRight()
    {
        servoZPosition += modifier

        if(servoZPosition > servoZOpen)
            servoZPosition = servoZOpen

        servoZ.position = servoZPosition
    }

    fun rotateLeft()
    {
        servoZPosition -= modifier

        if(servoZPosition < servoZClose)
            servoZPosition = servoZClose

        servoZ.position = servoZPosition
    }

    fun moveIn()
    {
        servoY.position = servoYIn
        servoYPosition = servoYIn
        servoZ.position = servoZIn
        servoZPosition = servoZIn
    }

    fun moveOut()
    {
        servoY.position = servoYOut
        servoYPosition = servoYOut
        servoZ.position = servoZOut
        servoZPosition = servoZOut
    }

    fun moveDeliver()
    {
        servoY.position = servoYD
        servoYPosition = servoYD
        servoZ.position = servoZD
        servoZPosition = servoZD
    }

    fun moveSpecifiedPosition(y: Double, z: Double)
    {
        servoY.position = y
        servoZ.position = z
    }

    fun stop(){
        stopMeasure()
    }
}