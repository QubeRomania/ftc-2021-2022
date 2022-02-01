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
import org.firstinspires.ftc.teamcode.tests.augmentedDrive.PoseStorage

@Autonomous
@Config
class AutoBlueLeft : AutoBase() {

    private val startPose = Pose2d(12.0, 64.0, Math.toRadians(-90.0))
    private val shippingHub = Pose2d(21.0,81.0, Math.toRadians(65.0))
    private val wallPose = Pose2d(14.0,59.0, Math.toRadians(-175.0))
    private val freightPose = Pose2d(-21.0,59.0,Math.toRadians(-175.0))



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

        cycleFreight(3)

        //Parking in storage
        var trajectory2 = drive.trajectoryBuilder(wallPose)
                .strafeTo(Vector2d(freightPose.x+3,freightPose.y))
                .build()

        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .lineToLinearHeading(Pose2d(wallPose.x-8,wallPose.y,wallPose.heading))
                .addDisplacementMarker{
                    drive.followTrajectoryAsync(trajectory2)
                }
                .build()
        drive.followTrajectory(trajectory1)

        PoseStorage.currentPose = drive.poseEstimate

    }

    private fun cycleFreight(n: Int){
        for (i in 1 until n)
        {
            var trajectory2 = drive.trajectoryBuilder(Pose2d(wallPose.x + 1,wallPose.y-2.0*i,wallPose.heading))
                    .strafeTo(Vector2d(freightPose.x - i*4.3 +4.3,freightPose.y-2.0),
                            SampleMecanumDrive.getVelocityConstraint(45.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build()

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker{
                        hw.outtake.closeSlider()
                        startIntake()
                    }
                    .lineToLinearHeading(Pose2d(wallPose.x + 1,wallPose.y-2.0*i,wallPose.heading))
                    .addDisplacementMarker{
                        drive.followTrajectoryAsync(trajectory2)
                    }
                    .build()

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

            drive.followTrajectory(trajectory1)

            waitMillis(1)

            drive.followTrajectory(
                    drive.trajectoryBuilder(
                            Pose2d(drive.poseEstimate.x,drive.poseEstimate.y,drive.poseEstimate.heading),true)
                            .addDisplacementMarker{
                                reverseIntake()
                            }
                            .strafeTo(Vector2d(wallPose.x,wallPose.y-2*i))
                            .addDisplacementMarker{
                                hw.outtake.openSlider()
                            }
                            .splineTo(Vector2d(shippingHub.x+1*i-1,shippingHub.y-0.5),shippingHub.heading)
                            .addDisplacementMarker{
                                hw.outtake.releaseServo()
                            }
                            .build()
            )

            waitMillis(300)
            hw.outtake.closeServo()
        }

        //Cycle freight for last time, changing the pickup position
        var trajectory2 = drive.trajectoryBuilder(Pose2d(wallPose.x,wallPose.y-2*3,wallPose.heading))
                .strafeTo(Vector2d(freightPose.x+5,freightPose.y-2*3))
                .splineToConstantHeading(Vector2d(freightPose.x+7,freightPose.y+15),freightPose.heading,
                        SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build()

        //Using async following for better auto
        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .lineToLinearHeading(Pose2d(wallPose.x,wallPose.y-2*3,wallPose.heading))
                .addDisplacementMarker{
                    drive.followTrajectoryAsync(trajectory2)
                }
                .build()
        drive.followTrajectory(trajectory1)

        waitMillis(1)

        //Delivering freight
        drive.followTrajectory(
                drive.trajectoryBuilder(
                        drive.poseEstimate,true)
                        .strafeTo(Vector2d(freightPose.x,freightPose.y-6.0))
                        .build()
        )

        drive.followTrajectory(
                drive.trajectoryBuilder(
                        drive.poseEstimate,true)
                        .addDisplacementMarker{
                            reverseIntake()
                        }
                        .strafeTo(Vector2d(wallPose.x,wallPose.y-2*3))
                        .addDisplacementMarker{
                            hw.outtake.openSlider()
                            //stopIntake()
                        }
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y),shippingHub.heading)
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(350)
        hw.outtake.closeServo()
    }

}