package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
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

        var intakeScale = 0.8

        var driveScale = 0.8
        var slowScale = 0.6

        others.closeServos()

        while(opModeIsActive()){
            val power = speed
            val rotPower = rotation

            if(gp2.right_stick_y.absoluteValue > 0.1)
                intake.setIntakePower(gp2.right_stick_y.toDouble() * intakeScale)
            outtake.moveSlider((gp2.right_trigger - gp2.left_trigger).toDouble())

            if(gp2.checkToggle(Gamepad.Button.RIGHT_BUMPER)) {
                if(!isUp) {
                    outtake.openSlider()
                    isUp = true
                }
                else{
                    outtake.closeServo()
                    outtake.closeSlider()
                    isUp = false
                    isLow = false
                    isMid = false
                }
            }

            if(gp2.checkToggle(Gamepad.Button.LEFT_BUMPER)) {
                if(!isLow) {
                    outtake.openLowSlider()
                    isLow = true
                    isUp = true
                    isMid = false
                }
                else if(!isMid){
                    outtake.openMidSlider()
                    isMid = true
                    isLow = false
                    isUp = true
                }
            }

            if(gp1.checkHold(Gamepad.Button.RIGHT_BUMPER))
                hw.motors.move(direction, power*slowScale, rotPower*slowScale)
            else
                hw.motors.move(direction, power*driveScale, rotPower*driveScale)

            if(gp2.checkToggle(Gamepad.Button.X))
            {
                carousel.carouselMotor.power = -0.3
                waitMillis(900)
                carousel.carouselMotor.power = -0.5
                waitMillis(300)
                carousel.carouselMotor.power = -0.7
                waitMillis(300)
            }

            if(gp2.checkToggle(Gamepad.Button.A) && outtake.outtakePosition > 10) {
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

            if(gp1.right_trigger > 0.1)
                customArm.extendMeasure()
            else
            {
                if(gp1.left_trigger > 0.1)
                    customArm.shrinkMeasure()
                else
                    customArm.stopMeasure()
            }


            if(gp1.checkToggle(Gamepad.Button.DPAD_UP))
                customArm.moveUp()
            if(gp1.checkToggle(Gamepad.Button.DPAD_DOWN))
                customArm.moveDown()

            if(gp1.checkToggle(Gamepad.Button.DPAD_RIGHT))
                customArm.rotateRight()
            if(gp1.checkToggle(Gamepad.Button.DPAD_LEFT))
                customArm.rotateLeft()

            //hw.motors.move(direction, power*driveScale, rotPower*driveScale)

            telemetry.addData("Outtake target", outtake.outtakePosition)
            telemetry.addData("Outtake power", outtake.outtakeSlider.power)
            telemetry.addData("PosPerpendicular",others.currentPosPerpendicular)
            telemetry.addData("PosParallel",others.currentPosParallel)
            telemetry.addData("Has Freight",outtake.hasFreight())
            telemetry.addData("Position sensor", outtake.distanceSensor.getDistance(DistanceUnit.CM))
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