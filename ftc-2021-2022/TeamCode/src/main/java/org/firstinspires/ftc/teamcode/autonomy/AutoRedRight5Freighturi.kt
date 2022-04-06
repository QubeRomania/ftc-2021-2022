package org.firstinspires.ftc.teamcode.autonomy

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
import org.firstinspires.ftc.teamcode.hardware.Outtake
import org.firstinspires.ftc.teamcode.tests.augmentedDrive.PoseStorage

@Autonomous
@Config
class AutoRedRight5Freighturi : AutoBase() {

    private val startPose = Pose2d(12.0, -64.0, Math.toRadians(90.0))
    private val shippingHub = Pose2d(21.0,-85.7, Math.toRadians(-65.0))
    private val wallPose = Pose2d(12.0,-63.5, Math.toRadians(180.0))
    private val freightPose = Pose2d(-29.0,-63.5,Math.toRadians(180.0))


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
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y+3.1),
                                shippingHub.heading)
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(250)
        hw.outtake.closeServo()

        cycleFreight(4)

        //Parking in storage
        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeServo()
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .addTemporalMarker(0.15){
                    hw.outtake.closeServo()
                }
                .splineTo(Vector2d(wallPose.x,wallPose.y+1.0),wallPose.heading)
                .lineTo(Vector2d(freightPose.x,freightPose.y+1.0))
                //.splineToLinearHeading(wallPose,wallPose.heading)
                .build()

        drive.followTrajectory(trajectory1)

        PoseStorage.currentPose = drive.poseEstimate

    }

    private fun cycleFreight(n: Int){
        for (i in 1 until n)
        {
            /*var trajectory2 = drive.trajectoryBuilder(wallPose)
                    .splineTo(Vector2d(freightPose.x - i*7.5 +7.5,freightPose.y),freightPose.heading,
                            SampleMecanumDrive.getVelocityConstraint(45.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build()

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker{
                        hw.outtake.closeSlider()
                        startIntake()
                    }
                    .lineToLinearHeading(wallPose)
                    .addDisplacementMarker{
                        drive.followTrajectoryAsync(trajectory2)
                    }
                    .build()
             */

            /*drive.followTrajectory(
                    drive.trajectoryBuilder(drive.poseEstimate)
                            .addDisplacementMarker{
                                hw.outtake.closeSlider()
                                startIntake()
                            }
                            .splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                            //.splineToLinearHeading(wallPose,wallPose.heading)
                            .splineTo(Vector2d(freightPose.x - i*2.75+2.75,freightPose.y),freightPose.heading,
                                    SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build()
            )*/

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker{
                        hw.outtake.closeServo()
                        hw.outtake.closeSlider()
                        startIntake()
                    }
                    .addTemporalMarker(0.15){
                        hw.outtake.closeServo()
                    }
                    .splineToSplineHeading(wallPose,wallPose.heading)
                    //.splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                    .lineTo(Vector2d(freightPose.x - i*2.4 + 2.4,freightPose.y))
                    //.splineToLinearHeading(wallPose,wallPose.heading)
                    .build()

            drive.followTrajectory(trajectory1)

            waitMillis(1)

            drive.followTrajectory(
                    drive.trajectoryBuilder(
                            Pose2d(drive.poseEstimate.x,drive.poseEstimate.y,drive.poseEstimate.heading),true)
                            .addDisplacementMarker{
                                reverseIntake()
                            }
                            .strafeTo(Vector2d(wallPose.x,wallPose.y))
                            .addDisplacementMarker{
                                hw.outtake.openSlider()
                            }
                            .splineTo(Vector2d(shippingHub.x-1.0-0.8*i+0.8,shippingHub.y+0.2+0.5*i-0.5),shippingHub.heading)
                            .addDisplacementMarker{
                                hw.outtake.releaseServo()
                            }
                            .build()
            )

            waitMillis(250)
            hw.outtake.closeServo()
        }

        //Cycle freight for last time, changing the pickup position

        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeServo()
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .addTemporalMarker(0.15){
                    hw.outtake.closeServo()
                }
                .splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                .lineTo(Vector2d(freightPose.x - 4*2.4 + 2.4,freightPose.y))
                .splineToConstantHeading(Vector2d(freightPose.x-8,freightPose.y-15),freightPose.heading,
                        SampleMecanumDrive.getVelocityConstraint(45.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToLinearHeading(wallPose,wallPose.heading)
                .build()
        drive.followTrajectory(trajectory1)

        waitMillis(1)

        //Delivering freight
        drive.followTrajectory(
                drive.trajectoryBuilder(
                        drive.poseEstimate,true)
                        .addTemporalMarker(1.0) {
                            reverseIntake()
                        }
                        .strafeTo(Vector2d(freightPose.x,freightPose.y))
                        .build()
        )

        drive.followTrajectory(
                drive.trajectoryBuilder(
                        drive.poseEstimate,true)
                        .strafeTo(Vector2d(wallPose.x,wallPose.y))
                        .addDisplacementMarker{
                            hw.outtake.openSlider()
                            //stopIntake()
                        }
                        .splineTo(Vector2d(shippingHub.x-1,shippingHub.y-1.5),shippingHub.heading,
                                SampleMecanumDrive.getVelocityConstraint(50.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(250)
        hw.outtake.closeServo()
    }

}