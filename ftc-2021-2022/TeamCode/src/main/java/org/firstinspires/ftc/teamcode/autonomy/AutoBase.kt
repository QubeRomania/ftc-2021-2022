package org.firstinspires.ftc.teamcode.autonomy

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer
import org.firstinspires.ftc.teamcode.waitMillis

@Config
abstract class AutoBase : org.firstinspires.ftc.teamcode.OpMode() {
    val drive : SampleMecanumDrive by lazy {
        SampleMecanumDrive(hardwareMap)
    }

    val intakePower = -1.0

    override fun preInit() {
        //telemetry.addLine("Initializing")
        //telemetry.update()

        //Start Streaming
    }

    override fun preInitLoop() {
        telemetry.addLine("Waiting for start...")
        telemetry.update()
    }

    fun startIntake() {
        hw.intake.setIntakePower(intakePower)
    }

    fun stopIntake() {
        hw.intake.stopIntake()
    }

    fun reverseIntake() {
        hw.intake.setIntakePower(-intakePower)
    }

    fun placeFreight() {
        hw.outtake.releaseServo()
        waitMillis(600)
        hw.outtake.closeServo()
    }

    /**
     * Camera related code
     * -------------------------------------------------------------
     * */


}