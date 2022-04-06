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
class AutoRed5Freight : AutoBase() {

    private val startPose = Pose2d(0.0,0.0,0.0)
    private val shippingHub = Pose2d(-14.0,20.0, Math.toRadians(-70.0))
    private val wallPose = Pose2d(0.0,0.0, Math.toRadians(0.0))
    private val freightPose = Pose2d(30.0,0.0,Math.toRadians(0.0))

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
                        .lineToLinearHeading(shippingHub)
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        telemetry.addData("Current Pose", "x : %.2f, y : %.2f, heading %.2f", drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)
        telemetry.update()

        waitMillis(105)

        cycleFreight(4)

        //Parking in storage
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            hw.outtake.outtakeSlider.targetPosition = -30
                            hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                            hw.outtake.outtakeSlider.power = 0.8
                            hw.outtake.closeServo()
                            startIntake()
                        }
                        .splineTo(Vector2d(-10.0,5.0),0.0)
                        .splineToConstantHeading(Vector2d(-1.0,0.5),0.0)
                        .lineTo(Vector2d(25.0-1,0.5))
                        .build()
        )
        PoseStorage.currentPose = drive.poseEstimate

    }

    private fun cycleFreight(n: Int){
        for (i in 1..n)
        {
            /*var trajectory2 = drive.trajectoryBuilder(wallPose)
                    .splineTo(Vector2d(freightPose.x + i*2.5 - 2.5,freightPose.y),freightPose.heading,
                            SampleMecanumDrive.getVelocityConstraint(50.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build()

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker{
                        hw.outtake.closeSlider()
                        hw.outtake.closeServo()
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

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.poseEstimate)
                            .addDisplacementMarker{
                                hw.outtake.outtakeSlider.targetPosition = -10*i+10
                                hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                                hw.outtake.outtakeSlider.power = 0.8
                                hw.outtake.closeServo()
                                startIntake()
                            }
                            .splineTo(Vector2d(-10.0,5.0),0.0)
                            .splineToConstantHeading(Vector2d(-1.0,i-1.0),0.0)
                            .lineTo(Vector2d(29.0+i-1,i-1.0))
                            .build()
            )

            telemetry.addData("Current Pose", "x : %.2f, y : %.2f, heading %.2f", drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)
            telemetry.update()

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

                            .lineTo(Vector2d(wallPose.x-2*i+1,wallPose.y+i-1))
                            .addDisplacementMarker{
                                reverseIntake()
                                hw.outtake.openSlider()
                            }
                            .splineToConstantHeading(Vector2d(-7.0,8.0),Math.toRadians(180.0))
                            .splineTo(Vector2d(shippingHub.x-1,shippingHub.y+3.0),shippingHub.heading+Math.toRadians(180.0))
                            .addDisplacementMarker{
                                hw.outtake.releaseServo()
                            }
                            .build()
            )

            telemetry.addData("Current Pose", "x : %.2f, y : %.2f, heading %.2f", drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)
            telemetry.update()

            waitMillis(105)
        }
    }

}