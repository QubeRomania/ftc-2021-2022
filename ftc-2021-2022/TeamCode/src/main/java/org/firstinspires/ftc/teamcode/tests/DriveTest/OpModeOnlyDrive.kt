package org.firstinspires.ftc.teamcode.tests.DriveTest

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime

abstract class OpModeOnlyDrive: LinearOpMode() {

    protected val hw by lazy {
        HardwareOnlyDrive(hardwareMap)
    }

    final override fun runOpMode() {
        hw.stop()

        preInit()

        preInitLoop()

        waitForStart()

        if (!opModeIsActive())
            return

        hw.run()

        hw.stop()
    }

    open fun preInit() {}

    open fun preInitLoop() {}

    //Runs the op mode.
    abstract fun HardwareOnlyDrive.run()
}

fun LinearOpMode.waitMillis(millis: Long) {
    val timer = ElapsedTime()
    while (opModeIsActive() && timer.milliseconds() <= millis)
        idle()
}