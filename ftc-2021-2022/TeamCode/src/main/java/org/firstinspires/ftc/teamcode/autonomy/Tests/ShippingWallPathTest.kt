package org.firstinspires.ftc.teamcode.autonomy.Tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.Hardware
import org.firstinspires.ftc.teamcode.waitMillis
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.autonomy.AutoBase
import org.firstinspires.ftc.teamcode.hardware.Outtake
import org.firstinspires.ftc.teamcode.tests.augmentedDrive.PoseStorage

@Autonomous
@Config
class ShippingWallPathTest : AutoBase() {

    private val startPose = Pose2d(12.0, -64.0, Math.toRadians(90.0))
    private val shippingHub = Pose2d(21.0,-84.7, Math.toRadians(-65.0))
    private val wallPose = Pose2d(9.0,-64.0, Math.toRadians(180.0))
    private val freightPose = Pose2d(-22.0,-64.0,Math.toRadians(180.0))


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
                                shippingHub.heading)
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(250)
        hw.outtake.closeServo()

        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeServo()
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .splineTo(Vector2d(wallPose.x+10,wallPose.y-6),Math.toRadians(140.0))
                .splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                .lineTo(Vector2d(freightPose.x,freightPose.y))
                //.splineToLinearHeading(wallPose,wallPose.heading)
                .build()

        drive.followTrajectory(trajectory1)

        /*drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            hw.outtake.closeSlider()
                            startIntake()
                        }
                        .splineTo(Vector2d(wallPose.x+8,wallPose.y-5),Math.toRadians(150.0))
                        .splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                        .splineTo(Vector2d(freightPose.x,freightPose.y),freightPose)
                        //.splineToLinearHeading(wallPose,wallPose.heading)
                        .build()
        )*/


        //Parking in storage

    }


}