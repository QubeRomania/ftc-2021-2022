package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
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
        var SLIDER_CLOSE = 0
        const val SLIDER_AUX = -100
        var SLIDER_START_POSITION = 0
        const val MULTIPLIER = 3

        const val OUTTAKE_POWER = 0.8

        val servoOpen = 0.59
        val servoClose = 0.82

        val necessaryDistance = 8.5
    }

    val outtakeSlider = hwMap.dcMotor["outtakeSlider"] ?: throw Exception("Failed to find motor outtakeSlider")

    val distanceSensor = hwMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") ?: throw Exception("Failed to find Rev2mDistanceSensor distanceSensor")
    val touchSensor = hwMap.get(RevTouchSensor::class.java,"touchSensor") ?: throw Exception("Failed to find RevTouchSensor touchSensor")

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
        outtakeSlider.power = OUTTAKE_POWER
    }

    fun closeSliderWithSensor() {

        outtakePosition = SLIDER_CLOSE
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = OUTTAKE_POWER

        while (!touchSensor.isPressed)
        {
            outtakePosition -= 10
            outtakeSlider.targetPosition = outtakePosition
            outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
            outtakeSlider.power = OUTTAKE_POWER
        }

        outtakeSlider.power = 0.0

        outtakePosition = 0

        outtakeSlider.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun closeSlider() {
        outtakePosition = SLIDER_CLOSE
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = OUTTAKE_POWER
    }

    fun openLowSlider() {
        outtakePosition = SLIDER_LOW
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = OUTTAKE_POWER
    }

    fun openMidSlider() {
        outtakePosition = SLIDER_MEDIUM
        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = OUTTAKE_POWER
    }

    fun moveSlider(power: Double) {
        outtakePosition += (power * MULTIPLIER).toInt()
        outtakePosition += SLIDER_START_POSITION

        if(outtakePosition > SLIDER_HIGH)
            outtakePosition = SLIDER_HIGH
        if(outtakePosition < SLIDER_AUX)
            outtakePosition = SLIDER_AUX

        outtakeSlider.targetPosition = outtakePosition
        outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        outtakeSlider.power = OUTTAKE_POWER
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

    fun hasFreight(): Boolean {
        if(distanceSensor.getDistance(DistanceUnit.CM) < necessaryDistance)
            return true
        return false
    }
}