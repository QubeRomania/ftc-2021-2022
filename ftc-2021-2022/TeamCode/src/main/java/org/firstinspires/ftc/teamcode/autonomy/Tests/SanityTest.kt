package org.firstinspires.ftc.teamcode.autonomy.Tests

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.teamcode.autonomy.AutoBase
import org.firstinspires.ftc.teamcode.hardware.Hardware
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.waitMillis

@Autonomous
class SanityTest : AutoBase() {

    override fun preInit() {
        super.preInit()
        telemetry.addLine("This works")
        telemetry.update()
    }

    override fun Hardware.run() {
        //waitForStart()
    }
}