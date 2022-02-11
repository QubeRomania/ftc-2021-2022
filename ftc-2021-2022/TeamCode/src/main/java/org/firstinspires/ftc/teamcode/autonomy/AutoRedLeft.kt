package org.firstinspires.ftc.teamcode.autonomy

import android.view.animation.LinearInterpolator
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.hardware.Hardware
import org.firstinspires.ftc.teamcode.waitMillis

@Autonomous
@Config
class AutoRedLeft : AutoBase() {

    private val startPose = Pose2d(-36.0, -64.0, Math.toRadians(90.0))
    private val shippingHub = Pose2d(-50.0,-88.0,Math.toRadians(-130.0))
    private val prepareDuck = Pose2d(-12.0,-76.0, Math.toRadians(100.0))
    private val deliverDuck = Pose2d(-12.0,-68.0,Math.toRadians(100.0))
    private val checkPoint1 = Pose2d(-36.0,-64.0,Math.toRadians(100.0))
    private val parkingPos = Pose2d(-12.0,-91.0,Math.toRadians(0.0))

    override fun preInit() {
        super.preInit()
        telemetry.addLine("Initializing...")
        telemetry.update()
        drive.poseEstimate = startPose
    }

    override fun Hardware.run() {
        //Stop streaming
        webcam.stopStreaming()

        telemetry.addData("Current Pose", "x : %.2f, y : %.2f, heading %.2f", drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)
        telemetry.update()

        drive.followTrajectory(
                drive.trajectoryBuilder(startPose,true)
                        .addDisplacementMarker{
                            openSliderSpecificPosition()
                        }
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y),
                                shippingHub.heading).build()
        )

        placeFreight()

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            hw.outtake.closeSlider()
                        }
                        .lineToLinearHeading(prepareDuck)
                        .build()
        )

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .strafeTo(Vector2d(deliverDuck.x,deliverDuck.y))
                        .build()
        )

        carousel.carouselMotor.power = -0.4
        waitMillis(2000)

        hw.carousel.stop()

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            startIntake()
                        }
                        .strafeTo(Vector2d(checkPoint1.x,checkPoint1.y))
                        .build()
        )

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate,true)
                        .addDisplacementMarker{
                            hw.outtake.openSlider()
                            stopIntake()
                        }
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y),shippingHub.heading)
                        .build()
        )

        placeFreight()

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            hw.outtake.closeSlider()
                        }
                        .lineToLinearHeading(parkingPos)
                        .build()
        )

    }

}