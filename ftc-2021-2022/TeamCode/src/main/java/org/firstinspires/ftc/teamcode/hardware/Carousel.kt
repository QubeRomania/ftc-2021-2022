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
 * Carousel subsystem.
 *
 * This class controls the hardware for delivering ducks
 */

class Carousel(hwMap: HardwareMap) {
    companion object {
        const val motorPower = 1.0
    }

    val carouselMotor = hwMap.dcMotor["carouselMotor"] ?: throw Exception("Failed to find motor carouselMotor")

    init {
        carouselMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        carouselMotor.power = 0.0
    }

    fun deliverDuck(sign: Int) {
        carouselMotor.power = sign * motorPower
    }

    fun moveCarousel(power: Double) {
        carouselMotor.power = power
    }

    fun stop() {
        carouselMotor.power = 0.0
    }
}