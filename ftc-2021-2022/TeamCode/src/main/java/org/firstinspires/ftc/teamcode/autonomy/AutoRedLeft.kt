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
    private val shippingHub = Pose2d(-27.0,-43.0,Math.toRadians(60.0))

    override fun preInit() {
        super.preInit()
        telemetry.addLine("Initializing...")
        telemetry.update()
        drive.poseEstimate = startPose
    }

    override fun preInitLoop() {
        super.preInitLoop()
    }

    override fun Hardware.run() {
        //Stop streaming

        telemetry.addData("Current Pose", "x : %.2f, y : %.2f, heading %.2f", drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)
        telemetry.update()

        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(startPose,true)
                        .addDisplacementMarker{
                            hw.outtake.openSlider()
                        }
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y),
                                shippingHub.heading).build()
        )

        placeFreight();

    }

}