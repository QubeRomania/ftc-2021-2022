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
class AutoRBlueRightNew : AutoBase() {

    private val startPose = Pose2d(-36.0, 64.0, Math.toRadians(-90.0))
    private val shippingHub = Pose2d(-50.0,88.0,Math.toRadians(130.0))
    private val prepareDuck = Pose2d(-16.0,105.0, Math.toRadians(0.0))
    private val deliverDuck = Pose2d(-16.0,62.0,Math.toRadians(-190.0))
    private val checkPoint1 = Pose2d(-36.0,64.0,Math.toRadians(-190.0))
    private val parkingPos = Pose2d(-17.0,90.0,Math.toRadians(0.0))

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
                        .lineToLinearHeading(Pose2d(-16.0,100.0,Math.toRadians(0.0)))
                        .build()
        )

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            openSliderSpecificPosition()
                        }
                        .lineTo(Vector2d(-45.0,101.0))
                        .build()
        )

        placeFreight()

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

        carousel.carouselMotor.power = 0.6
        waitMillis(1500)
        carousel.carouselMotor.power = 0.8
        waitMillis(1000)

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .lineTo(Vector2d(parkingPos.x,parkingPos.y-2.0))
                        .build()
        )

    }

}