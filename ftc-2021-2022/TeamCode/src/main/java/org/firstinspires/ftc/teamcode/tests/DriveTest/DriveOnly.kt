package org.firstinspires.ftc.teamcode.tests.DriveTest

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Gamepad

@TeleOp(name = "DriveOnly", group = "Main")
class DriveOnly: OpModeOnlyDrive() {

    override fun preInit() {}

    override fun preInitLoop() {
        telemetry.addLine("Waiting for start...")
        telemetry.update()
        idle()
    }

    override fun HardwareOnlyDrive.run() {
        val gp1 = Gamepad(gamepad1)


        while (opModeIsActive())
        {
            val power = speed
            val rotPower = rotation

            hw.motors.move(direction, power, rotPower)
        }
    }


    ///The direction in which the robot is translating
    private val direction: Double
        get() {
            val x = gamepad1.left_stick_x.toDouble()
            val y = -gamepad1.left_stick_y.toDouble()

            return Math.atan2(y, x) / Math.PI * 180.0 - 90.0
        }

    /// Rotation around the robot's Z axis.
    private val rotation: Double
        get() = -gamepad1.right_stick_x.toDouble()

    /// Translation speed.
    private val speed: Double
        get() {
            val x = gamepad1.left_stick_x.toDouble()
            val y = gamepad1.left_stick_y.toDouble()

            return Math.sqrt((x * x) + (y * y))
        }
}