package org.firstinspires.ftc.teamcode.autonomy

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.Outtake
import org.firstinspires.ftc.teamcode.waitMillis
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Config
abstract class AutoBase : org.firstinspires.ftc.teamcode.OpMode() {
    val drive : SampleMecanumDrive by lazy {
        SampleMecanumDrive(hardwareMap)
    }

    val intakePower = -1.0

    override fun preInit() {
        //telemetry.addLine("Initializing")
        //telemetry.update()

        //Start Streaming
        webcam.openCameraDevice()
        webcam.setPipeline(pipeline)
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    override fun preInitLoop() {
        position = pipeline.analysis
        telemetry.addData("Team Element position", position)
        telemetry.update()
        telemetry.addLine("Waiting for start...")
        telemetry.update()
    }

    fun startIntake() {
        hw.intake.setIntakePower(intakePower)
    }

    fun stopIntake() {
        hw.intake.stopIntake()
    }

    fun openSliderSpecificPosition()
    {
        when (position) {
            IgnitePipeline.FreightFrenzyPosition.RIGHT -> hw.outtake.openSlider()
            IgnitePipeline.FreightFrenzyPosition.LEFT -> hw.outtake.openLowSlider()
            IgnitePipeline.FreightFrenzyPosition.CENTER -> hw.outtake.openMidSlider()
        }
    }

    fun reverseIntake() {
        hw.intake.setIntakePower(-intakePower)
    }

    fun placeFreight() {
        hw.outtake.releaseServo()

        waitMillis(600)
        hw.outtake.closeServo()
    }

    fun placeDuck() {
        hw.outtake.outtakeServo.position = 0.55
        hw.outtake.outtakeSlider.targetPosition = 820
        hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.outtake.outtakeSlider.power = Outtake.OUTTAKE_POWER
        waitMillis(600)
        hw.outtake.closeServo()
        hw.outtake.outtakeSlider.targetPosition = 880
        hw.outtake.outtakeSlider.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.outtake.outtakeSlider.power = Outtake.OUTTAKE_POWER
        sleep(500)
    }

    /**
     * Camera related code
     * -------------------------------------------------------------
     * */

    val webcam: OpenCvCamera by lazy {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
    }

    var pipeline = IgnitePipeline(telemetry)
    var position = pipeline.analysis
}