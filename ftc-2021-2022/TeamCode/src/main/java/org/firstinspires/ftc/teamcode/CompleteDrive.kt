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
        var isUp = false
        var isMid = false
        var isLow = false
        var isOpenedArm = false

        var intakeScale = 0.7

        var driveScale = 0.8

        others.closeServos()

        while(opModeIsActive()){
            val power = speed
            val rotPower = rotation

            if(gp2.right_stick_y.absoluteValue > 0.1)
                intake.setIntakePower(gp2.right_stick_y.toDouble() * intakeScale)
            else intake.setIntakePower((-gp1.left_trigger.toDouble() + gp1.right_trigger)*intakeScale)
            outtake.moveSlider((gp2.right_trigger - gp2.left_trigger).toDouble())

            if(gp2.checkToggle(Gamepad.Button.RIGHT_BUMPER)) {
                if(!isUp) {
                    outtake.openSlider()
                    isUp = true
                }
                else{
                    outtake.closeSlider()
                    outtake.closeServo()
                    isUp = false
                }

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

            carousel.moveCarousel(gp2.left_stick_x.toDouble())

            customArm.moveArm(-gp1.left_trigger.toDouble() + gp1.right_trigger)

            if(gp1.checkToggle(Gamepad.Button.RIGHT_BUMPER))
            {
                if(!isOpenedArm){
                    isOpenedArm = true
                    customArm.openArm()
                }
                else {
                    isOpenedArm = false
                    customArm.closeArm()
                }
            }

            hw.motors.move(direction, power*driveScale, rotPower*driveScale)

            telemetry.addData("Outtake target", outtake.outtakePosition)
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