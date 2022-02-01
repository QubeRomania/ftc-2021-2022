package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.io.File
import java.io.FileOutputStream
import java.io.PrintWriter
import java.lang.Exception
import java.util.*
import kotlin.math.absoluteValue


/**
 * OutTake subsystem.
 *
 * This class controls the hardware for placing freight
 */
class Outtake(hwMap: HardwareMap) {
    companion object {
        const val SLIDER_LOW = 320
        const val SLIDER_MEDIUM = 600
        const val SLIDER_HIGH = 870
        const val SLIDER_CLOSE = 0
        var SLIDER_START_POSITION = 0
        const val MULTIPLIER = 3

        val servoOpen = 0.55
        val servoClose = 0.82
    }

    val outtakeSlider = hwMap.dcMotor["outtakeSlider"] ?: throw Exception("Failed to find motor outtakeSlider")

    var outtakePosition: Int = 0

    val outtakeServo = hwMap.servo["outtakeServo"] ?: throw Exception("Failed to find servo outtakeServo")

    init {
        outtakeSlider.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        outtakeSlider.direction = DcMotorSimple.Direction.FORWARD
        outtakeSlider.mode = DcMotor.RunMode.RUN_USING_ENCODER
        outtakeSlider.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER


        outtakeSlider.power = 0.0

        outtakePosition = 0

        closeServo()
    }

    fun openSlider() {
        outtakePosition = SLIDER_HIGH
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = 1.0
    }

    fun closeSlider() {

        outtakePosition = SLIDER_CLOSE
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = 1.0
    }

    fun openLowSlider() {
        outtakePosition = SLIDER_LOW
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = 1.0
    }

    fun openMidSlider() {
        outtakePosition = SLIDER_MEDIUM
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = 1.0
    }

    fun moveSlider(power: Double) {
        outtakePosition += (power * MULTIPLIER).toInt()
        outtakePosition += SLIDER_START_POSITION

        if(outtakePosition > SLIDER_HIGH)
            outtakePosition = SLIDER_HIGH
        if(outtakePosition < SLIDER_CLOSE)
            outtakePosition = SLIDER_CLOSE

        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = 1.0
    }

    fun releaseServo() {
        setServoPositions(servoOpen)
    }

    fun closeServo(){
        setServoPositions(servoClose)
    }

    fun stop(){
        outtakeSlider.power = 0.0
    }

    fun printPosition(telemetry: Telemetry) {
        telemetry.addLine("Position outtake")
                .addData("Slider","%d", outtakeSlider.currentPosition)
    }

    fun setPower(v: Double) {
        outtakeSlider.power = v
    }

    fun setServoPositions(pos: Double) {
        outtakeServo.position = pos
    }
}