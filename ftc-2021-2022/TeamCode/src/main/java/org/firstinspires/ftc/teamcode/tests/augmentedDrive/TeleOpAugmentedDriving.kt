package org.firstinspires.ftc.teamcode.tests.augmentedDrive

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Gamepad
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.hardware.Hardware
import java.lang.Math.atan2
import kotlin.math.absoluteValue
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder


@TeleOp(name = "Augmented Drive", group = "Augmented")
class TeleOpAugmentedDriving: OpMode() {

    val drive : SampleMecanumDriveCancelable by lazy {
        SampleMecanumDriveCancelable(hardwareMap)
    }

    enum class Mode
    {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    var currentMode = Mode.DRIVER_CONTROL

    private val wallPose = Pose2d(9.0,-62.35, Math.toRadians(180.0))
    private val shippingHub = Pose2d(21.0,-80.0, Math.toRadians(-60.0))

    override fun preInit() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        drive.poseEstimate = PoseStorage.currentPose
    }

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

        //others.closeServos()

        while(opModeIsActive()){

            when(currentMode) {
                Mode.DRIVER_CONTROL -> {
                    drive.setWeightedDrivePower(
                            Pose2d(
                                    (-gamepad1.right_stick_y).toDouble(),
                                    (-gamepad1.right_stick_x).toDouble(),
                                    (-gamepad1.left_stick_x).toDouble()
                            )
                    )
                    if (gp1.checkToggle(Gamepad.Button.A)) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.poseEstimate)
                                        .lineToLinearHeading(wallPose)
                                        .build()
                        )

                        currentMode = Mode.AUTOMATIC_CONTROL
                    }
                    if(gp1.checkToggle(Gamepad.Button.B)){
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.poseEstimate)
                                        .lineToLinearHeading(shippingHub)
                                        .build()

                        )

                        currentMode = Mode.AUTOMATIC_CONTROL
                    }
                }
                Mode.AUTOMATIC_CONTROL -> {
                    if(gp1.checkToggle(Gamepad.Button.X)){
                        drive.cancelFollowing()
                        currentMode = Mode.DRIVER_CONTROL
                    }

                    if(!drive.isBusy)
                        currentMode = Mode.DRIVER_CONTROL
                }
            }

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


            //hw.motors.move(direction, power*driveScale, rotPower*driveScale)

            telemetry.addData("Outtake target", outtake.outtakePosition)
            telemetry.addData("PosPerpendicular",others.currentPosPerpendicular)
            telemetry.addData("PosParallel",others.currentPosParallel)
            outtake.printPosition(telemetry)
            telemetry.update()
        }
    }
}