package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.Hardware
import java.lang.Math.atan2
import kotlin.math.absoluteValue

@TeleOp(name = "CompleteDrive", group = "Main")
class CompleteDrive: OpMode() {

    override fun preInit() {}

    override fun preInitLoop() {
        telemetry.addLine("Waiting for start...")
        telemetry.update()
        idle()
    }

    override fun Hardware.run() {
        val gp1 = Gamepad(gamepad1)
        val gp2 = Gamepad(gamepad2)

        var isPlacing = false
        var isDelivering = false

        while(opModeIsActive()){
            val power = speed
            val rotPower = rotation

            if(gp2.right_stick_y.absoluteValue > 0.1)
                intake.setIntakePower(gp2.right_stick_y.toDouble())
            else intake.setIntakePower(-gp1.left_trigger.toDouble() + gp1.right_trigger)
            outtake.moveSlider((gp2.right_trigger - gp2.left_trigger).toDouble())

            if(gp2.right_bumper) {
                outtake.openSlider()
            }

            if(gp2.left_bumper) {
                outtake.closeSlider()
            }

            if(gp2.checkToggle(Gamepad.Button.A) && outtake.outtakePosition > 0) {
                if(!isPlacing) {
                    isPlacing = true
                    outtake.releaseServo()
                }
                else {
                    isPlacing = false
                    outtake.closeServo()
                    //outtake.closeSlider()
                }
            }

            if(gp2.checkToggle(Gamepad.Button.X))
            {
                if(!isDelivering){
                    isDelivering = true
                    carousel.deliverDuck()
                }
                else {
                    isDelivering = true
                    carousel.stop()
                }
            }

            hw.motors.move(direction, power, rotPower)

            telemetry.addData("Outake target", outtake.outtakePosition)
            outtake.printPosition(telemetry)
            telemetry.update()
        }
    }

    ///The direction in which the robot is translating
    private val direction: Double
        get() {
            val x = gamepad1.left_stick_x.toDouble()
            val y = -gamepad1.left_stick_y.toDouble()

            return atan2(y, x) / Math.PI * 180.0 - 90.0
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