package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.Exception

/**
 * Intake subsystem
 *
 * This class controls the hardware which takes the freight
 */

class Intake(hwMap: HardwareMap) {
    val intakeMotor = hwMap.dcMotor.get("intakeMotor") ?: throw Exception("failed to find motor intakeMotor")

    init {
        intakeMotor.direction = DcMotorSimple.Direction.FORWARD
        stopIntake()
    }

    fun setIntakePower(power: Double){
        intakeMotor.power = power
    }

    fun stopIntake() {
        setIntakePower(0.0)
    }

    //-------------------------------------------------------------------------------------------------------

    fun stop() {
        stopIntake()
    }
}