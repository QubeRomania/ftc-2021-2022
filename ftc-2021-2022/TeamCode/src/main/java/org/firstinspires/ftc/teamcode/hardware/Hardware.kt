package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap

class Hardware(hwMap: HardwareMap) {
    val motors = DriveMotors(hwMap)
    val intake = Intake(hwMap)
    val outtake = Outtake(hwMap)
    val carousel = Carousel(hwMap)
    val customArm = CustomArm(hwMap)
    val others = Others(hwMap)

    fun stop(){
        motors.stop()
        intake.stop()
        outtake.stop()
        carousel.stop()
        customArm.stop()
        others.stop()
    }
}