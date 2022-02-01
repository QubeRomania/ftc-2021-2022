package org.firstinspires.ftc.teamcode.tests.DriveTest

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.DriveMotors

class HardwareOnlyDrive(hwMap: HardwareMap) {
    val motors = DriveMotors(hwMap)

    fun stop(){
        motors.stop()
    }
}