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
class AutoRedRight : AutoBase() {

    private val startPose = Pose2d(12.0, -64.0, Math.toRadians(90.0))
    private val shippingHub = Pose2d(20.0,-84.7, Math.toRadians(-65.0))
    private val wallPose = Pose2d(8.0,-62.5, Math.toRadians(180.0))
    private val freightPose = Pose2d(-30.0,-62.5,Math.toRadians(180.0))


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
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y+0.2),
                        shippingHub.heading,
                                SampleMecanumDrive.getVelocityConstraint(50.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(200)
        hw.outtake.closeServo()

        cycleFreight(3)

        //Parking in storage

        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.outtakeSlider.targetPosition = -20
                    hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                    hw.outtake.outtakeSlider.power = 0.8
                    startIntake()
                }
                .splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                .lineTo(Vector2d(freightPose.x,freightPose.y))
                .build()
        drive.followTrajectory(trajectory1)

        PoseStorage.currentPose = drive.poseEstimate

    }

    private fun cycleFreight(n: Int){
        for (i in 1 until n)
        {
            var trajectory2 = drive.trajectoryBuilder(wallPose)
                    .lineTo(Vector2d(freightPose.x - i*6 +6,freightPose.y+2.0),
                            SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build()

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker{
                        hw.outtake.closeSlider()
                        startIntake()
                    }
                    .lineToLinearHeading(Pose2d(wallPose.x,wallPose.y+2.0,wallPose.heading))
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

            var trajectory4 = drive.trajectoryBuilder(Pose2d(wallPose.x,wallPose.y+2.0,wallPose.heading),true)
                    .lineToLinearHeading(Pose2d(shippingHub.x,shippingHub.y-i*1+1,Math.toRadians(115.0)))
                    .addDisplacementMarker{
                        hw.outtake.releaseServo()
                    }
                    .build()

            var trajectory3 = drive.trajectoryBuilder(
                    Pose2d(drive.poseEstimate.x,drive.poseEstimate.y,drive.poseEstimate.heading),true)
                    .addTemporalMarker(0.1){
                        if(hw.outtake.hasFreight())
                            reverseIntake()
                    }
                    .addTemporalMarker(0.2){
                        if(hw.outtake.hasFreight())
                            reverseIntake()
                    }
                    .addTemporalMarker(0.3){
                        if(hw.outtake.hasFreight())
                            reverseIntake()
                    }
                    .addTemporalMarker(0.4){
                        if(hw.outtake.hasFreight())
                            reverseIntake()
                    }
                    .addTemporalMarker(0.5){
                        if(hw.outtake.hasFreight())
                            reverseIntake()
                    }
                    .addTemporalMarker(0.7){
                        reverseIntake()
                    }
                    .strafeTo(Vector2d(wallPose.x,wallPose.y+2.0))
                    .addDisplacementMarker{
                        reverseIntake()
                    }
                    .addDisplacementMarker{
                        hw.outtake.openSlider()
                        drive.followTrajectoryAsync(trajectory4)
                    }
                    .build()

            drive.followTrajectory(trajectory3)

            waitMillis(200)
            hw.outtake.closeServo()
        }

        //Cycle freight for last time, changing the pickup position
        var trajectory2 = drive.trajectoryBuilder(Pose2d(wallPose.x,wallPose.y+3.5,wallPose.heading))
                .lineTo(Vector2d(freightPose.x-3,freightPose.y+3.5))
                .addDisplacementMarker{
                    reverseIntake()
                }
                .build()

        //Using async following for better auto
        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.outtakeSlider.targetPosition = -30
                    hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                    hw.outtake.outtakeSlider.power = 0.8
                    startIntake()
                }
                .lineToLinearHeading(Pose2d(wallPose.x,wallPose.y+3.5,wallPose.heading))
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
                        .addTemporalMarker(0.1){
                            if(hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.2){
                            if(hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.3){
                            if(hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.4){
                            if(hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.5){
                                reverseIntake()
                        }
                        .strafeTo(Vector2d(wallPose.x,wallPose.y+3.5))
                        .addDisplacementMarker{
                            hw.outtake.openSlider()
                            //stopIntake()
                        }
                        .splineTo(Vector2d(shippingHub.x+1.0,shippingHub.y),shippingHub.heading,
                                SampleMecanumDrive.getVelocityConstraint(50.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(30.0))
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(200)
        hw.outtake.closeServo()
    }

}