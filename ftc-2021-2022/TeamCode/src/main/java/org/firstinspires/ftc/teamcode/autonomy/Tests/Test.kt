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
class Test : AutoBase() {

    private val startPose = Pose2d(12.0, -64.0, Math.toRadians(90.0))
    private val shippingHub = Pose2d(21.0,-85.7, Math.toRadians(-65.0))
    private val wallPose = Pose2d(10.0,-64.0, Math.toRadians(180.0))
    private val freightPose = Pose2d(-25.0,-64.0,Math.toRadians(180.0))


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
                        .splineTo(Vector2d(shippingHub.x,shippingHub.y+1.0),shippingHub.heading,
                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40.0))
                        .addDisplacementMarker{
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        waitMillis(105)
        hw.outtake.closeServo()

        for(i in 1..4) {

            var trajectory2 = drive.trajectoryBuilder(Pose2d(freightPose.x,freightPose.y-i*2.6+2.6,freightPose.heading),true)
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
                    .splineToConstantHeading(Vector2d(wallPose.x, wallPose.y-i*3.0+3.0),wallPose.heading)
                    .addDisplacementMarker{
                        reverseIntake()
                    }
                    .addDisplacementMarker()
                    {
                        hw.outtake.openSlider()
                    }
                    .splineToConstantHeading(Vector2d(15.0, -70.0), 0.0)
                    .splineTo(Vector2d(shippingHub.x+1*i-1, shippingHub.y-i*3.0), shippingHub.heading,
                            SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(40.0)
                    )
                    .addDisplacementMarker {
                        hw.outtake.releaseServo()
                    }
                    .build()

            var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                    .addDisplacementMarker {
                        hw.outtake.closeServo()
                        hw.outtake.closeSlider()
                        startIntake()
                    }
                    .addTemporalMarker(0.15) {
                        hw.outtake.closeServo()
                    }
                    .splineTo(Vector2d(16.0,-72.0),wallPose.heading)
                    .splineToConstantHeading(Vector2d(wallPose.x, wallPose.y-i*3.0+3.0), wallPose.heading)
                    //.splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                    .splineToConstantHeading(Vector2d(freightPose.x-4.5*i+4.0, freightPose.y-i*2.6+2.6),freightPose.heading)
                    .addDisplacementMarker{
                        drive.update()
                        drive.followTrajectoryAsync(trajectory2)
                    }
                    //.splineToLinearHeading(wallPose,wallPose.heading)
                    .build()

            drive.followTrajectory(trajectory1)

            drive.update()

            /*drive.followTrajectory(
                    drive.trajectoryBuilder(drive.poseEstimate, true)
                            .lineTo(Vector2d(wallPose.x, wallPose.y-i*2.5+2.5))
                            .addDisplacementMarker()
                            {
                                hw.outtake.openSlider()
                            }
                            .splineToConstantHeading(Vector2d(15.0, -70.0), 0.0)
                            .splineTo(Vector2d(shippingHub.x, shippingHub.y-i*2.5), shippingHub.heading,
                                    SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(35.0)
                                    )
                            .addDisplacementMarker {
                                hw.outtake.releaseServo()
                            }
                            .build()
            )*/
            waitMillis(105)
            hw.outtake.closeServo()
        }

        /*var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker {
                    hw.outtake.closeServo()
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .addTemporalMarker(0.15) {
                    hw.outtake.closeServo()
                }
                .splineToSplineHeading(Pose2d(wallPose.x, wallPose.y-4*2.6+2.6, wallPose.heading), wallPose.heading)
                //.splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                .lineTo(Vector2d(freightPose.x-4.5*4+4.0, freightPose.y-4*2.6+2.6))
                //.splineToLinearHeading(wallPose,wallPose.heading)
                .build()

        drive.followTrajectory(trajectory1)

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate, true)
                        .addTemporalMarker(0.1) {
                            if (hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.2) {
                            if (hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.3) {
                            if (hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.4) {
                            if (hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .addTemporalMarker(0.5) {
                            if (hw.outtake.hasFreight())
                                reverseIntake()
                        }
                        .lineTo(Vector2d(wallPose.x, wallPose.y - 4 * 2.6 + 2.6))
                        .addDisplacementMarker {
                            reverseIntake()
                        }
                        .addDisplacementMarker()
                        {
                            hw.outtake.openSlider()
                        }
                        .splineToConstantHeading(Vector2d(15.0, -70.0), 0.0)
                        .splineTo(Vector2d(shippingHub.x - 1 * 4 + 1, shippingHub.y - 4 * 2.6), shippingHub.heading,
                                SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(35.0)
                        )
                        .addDisplacementMarker {
                            hw.outtake.releaseServo()
                        }
                        .build()
        )

        /*drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate, true)
                        .lineTo(Vector2d(wallPose.x, wallPose.y-i*2.5+2.5))
                        .addDisplacementMarker()
                        {
                            hw.outtake.openSlider()
                        }
                        .splineToConstantHeading(Vector2d(15.0, -70.0), 0.0)
                        .splineTo(Vector2d(shippingHub.x, shippingHub.y-i*2.5), shippingHub.heading,
                                SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(35.0)
                                )
                        .addDisplacementMarker {
                            hw.outtake.releaseServo()
                        }
                        .build()
        )*/
        waitMillis(105)
        hw.outtake.closeServo()
        n
         */

        var trajectory3 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker {
                    hw.outtake.closeServo()
                    hw.outtake.outtakeSlider.targetPosition = -20
                    hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
                    hw.outtake.outtakeSlider.power = 0.8
                    startIntake()
                }
                .addTemporalMarker(0.15) {
                    hw.outtake.closeServo()
                    hw.customArm.moveSpecifiedPosition(0.35,0.2)
                }
                .splineToSplineHeading(Pose2d(wallPose.x, wallPose.y-5*3.0+3.0, wallPose.heading), wallPose.heading)
                //.splineTo(Vector2d(wallPose.x,wallPose.y),wallPose.heading)
                .lineTo(Vector2d(freightPose.x, freightPose.y-5*2.6+2.6),
                        SampleMecanumDrive.getVelocityConstraint(70.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToLinearHeading(wallPose,wallPose.heading)
                .build()

        drive.followTrajectory(trajectory3)
        /*drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            hw.outtake.closeSlider()
                        }
                        .splineTo(Vector2d(-5.0,0.0),0.0)
                        .lineTo(Vector2d(25.0,0.0))
                        .build()
        )
         */
        /*drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                        .addDisplacementMarker{
                            hw.outtake.closeSlider()
                        }
                        .lineTo(Vector2d(-13.0,15.0))
                        .splineTo(Vector2d(-6.0,1.0),0.0)
                        .lineTo(Vector2d(20.0,1.0))
                        .build()
        )
         */
        /*

        var trajectory1 = drive.trajectoryBuilder(drive.poseEstimate)
                .addDisplacementMarker{
                    hw.outtake.closeServo()
                    hw.outtake.closeSlider()
                    startIntake()
                }
                .lineTo(Vector2d(wallPose.x+5,wallPose.y-6))
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
*/

        //Parking in storage

    }


}