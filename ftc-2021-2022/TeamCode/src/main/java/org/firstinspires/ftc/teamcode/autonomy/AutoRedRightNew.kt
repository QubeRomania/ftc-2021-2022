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
class AutoRedRightNew : AutoBase() {

    private val startPose = Pose2d(12.0, -64.0, Math.toRadians(90.0))
    private val shippingHub = Pose2d(21.0,-84.7, Math.toRadians(-65.0))
    private val wallPose = Pose2d(9.0,-65.2, Math.toRadians(180.0))
    private val freightPose = Pose2d(-24.5,-65.2,Math.toRadians(180.0))

    private var k = 1.0


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

        waitMillis(105)

        cycleFreight(4)

        //Parking in storage
        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.outtakeSlider.targetPosition = -40
                    hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                    hw.outtake.outtakeSlider.power = 0.8
                }
                .addTemporalMarker(0.15){
                    hw.outtake.closeServo()
                }
                .splineTo(Vector2d(wallPose.x+10,wallPose.y-6),Math.toRadians(140.0))
                .splineTo(Vector2d(wallPose.x,wallPose.y-4.0),wallPose.heading)
                .lineTo(Vector2d(freightPose.x,freightPose.y-4.0))
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
                            .splineTo(Vector2d(wallPose.x+5,wallPose.y-4),wallPose.heading)
                            .splineToConstantHeading(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                            //.splineToLinearHeading(wallPose,wallPose.heading)
                            .splineTo(Vector2d(freightPose.x - i*7.5+7.5,freightPose.y),freightPose.heading,
                                    SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build()
            )

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.poseEstimate)
                            .addDisplacementMarker{
                                hw.outtake.closeSlider()
                                startIntake()
                            }
                            .splineTo(Vector2d(wallPose.x+8,wallPose.y-4),Math.toRadians(145.0))
                            .splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                            .splineTo(Vector2d(freightPose.x-i*2+2,freightPose.y),freightPose.heading)
                            //.splineToLinearHeading(wallPose,wallPose.heading)
                            .build()
            )*/

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker{
                        hw.outtake.closeServo()
                        if (i == 3 && i == 4)
                        {
                            hw.outtake.outtakeSlider.targetPosition = -35
                            hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                            hw.outtake.outtakeSlider.power = 0.8
                        }
                        hw.outtake.closeSlider()
                        startIntake()
                    }
                    .addTemporalMarker(0.15){
                        hw.outtake.closeServo()
                    }
                    .splineTo(Vector2d(wallPose.x+10,wallPose.y-6),Math.toRadians(140.0))
                    .splineTo(Vector2d(wallPose.x,wallPose.y - 0.5 * i + 0.5),wallPose.heading)
                    .lineTo(Vector2d(freightPose.x - i*2.4 + 2.4,freightPose.y -0.5 * i + 0.5))
                    //.splineToLinearHeading(wallPose,wallPose.heading)
                    .build()

            drive.followTrajectory(trajectory1)

            if(hw.outtake.hasFreight())
                reverseIntake()

            k =  when (i) {
                1 -> 3.3
                2 -> 4.0
                3 -> 7.2
                else -> 5.4
            }

            drive.followTrajectory(
                    drive.trajectoryBuilder(
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

                            .lineTo(Vector2d(wallPose.x,wallPose.y - 0.6 * i + 0.6))
                            .addDisplacementMarker{
                                reverseIntake()
                                hw.outtake.openSlider()
                            }
                            .splineTo(Vector2d(shippingHub.x+1,shippingHub.y - k+1),shippingHub.heading)
                            .addDisplacementMarker{
                                hw.outtake.releaseServo()
                            }
                            .build()
            )

            waitMillis(105)
        }

        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeServo()
                    hw.outtake.outtakeSlider.targetPosition = -35
                    hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                    hw.outtake.outtakeSlider.power = 0.8
                    startIntake()
                }
                .addTemporalMarker(0.15){
                    hw.outtake.closeServo()
                }
                .splineTo(Vector2d(wallPose.x+10,wallPose.y-6),Math.toRadians(140.0))
                .splineTo(Vector2d(wallPose.x,wallPose.y - 0.5 * 4 + 0.5),wallPose.heading)
                .lineTo(Vector2d(freightPose.x - 4*2.4 + 2.4,freightPose.y -0.5 * 4 + 0.5))
                //.splineToLinearHeading(wallPose,wallPose.heading)
                .build()

        drive.followTrajectory(trajectory1)

        drive.followTrajectory(
                drive.trajectoryBuilder(
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

                        .lineTo(Vector2d(wallPose.x,wallPose.y - 0.6 * 4 + 0.6))
                        .addDisplacementMarker{
                            reverseIntake()
                            hw.outtake.openSlider()
                        }
                        .splineTo(Vector2d(shippingHub.x+1,shippingHub.y - 9.4+1),shippingHub.heading)
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(105)
    }

}