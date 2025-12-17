/**
 * This program is used to play with and find useful settings for any camera to best detect AprilTags
 */

// PACKAGE STATEMENT
package org.firstinspires.ftc.teamcode.Utility_CameraCalibration;

// CORE FTC IMPORTS

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// MISCELLANEOUS FTC IMPORTS
import com.qualcomm.robotcore.util.ElapsedTime;

// VISION ANALYSIS FTC IMPORTS
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// DASHBOARD ACMEROBOTICS IMPORTS
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.concurrent.TimeUnit;

/*
 * Code starts here
 */
@Config
@TeleOp(name = "Utility_CameraCalibration", group = "Linear OpMode")
public class Utility_CameraCalibration extends LinearOpMode {

    // CAMERA & VISION VARIABLES
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int myExposure;
    private int minExposure;
    private int maxExposure;
    private int myGain;
    private int minGain;
    private int maxGain;

    // TELEMETRY VARIABLES
    private FtcDashboard dashboard;
    TelemetryPacket packet;

    // MISCELLANEOUS VARIABLES
    private ElapsedTime runtime = new ElapsedTime();
    public static int dashboardCameraFramerate = 0;
    private int lastDashboardCameraFramerate = dashboardCameraFramerate;

    // OP MODE
    @Override
    public void runOpMode() {
        // Print instructions to the driver hub
        handleInstructions();

        myGain=1;

        // Setups
        initializeCamera();
        getCameraSetting();
        initializeDashboard();

        // Wait here until the PLAY button is pressed on the driver hub
        waitForStart();
        runtime.reset();

        // Loops until the STOP button is pressed on the driver hub
        while (opModeIsActive()) {
            handleFramerateChanges();
            handleTelemetry();
        }
    }

    /**
     * Prints instructions to the driver hub telemetry screen, since most of the magic happens on the FTC Dashboard
     */
    void handleInstructions() {
        telemetry.addLine("An Opmode to Calibrate the Camera.");
        telemetry.addLine("- - -");
        telemetry.addLine("On a computer, connect to the Robot's wifi network, then go to 192.168.43.1:8080/dash");
        telemetry.addLine("Then, press PLAY on the driver hub to begin!");
        telemetry.update();
    }

    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, dashboardCameraFramerate);
    }

    private void initializeCamera() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the webcam vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Sends telemetry data to the FTC Dashboard
     */
    void handleTelemetry() {
        // Make a new, empty telemetry packet for us to fill
        packet = new TelemetryPacket();

        // Read sensors
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();


        // Send the telemetry
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    /**
     * If the user requests a different framerate on the FTCDashboard, we need to
     * restart the camera stream for it to take effect.
     * -
     * Faster framerates will make the main loop run slower, since it needs to send
     * the video feed to the dashboard. This means asking to view a lower framerate stream
     * will make your robot more responsive and better at detecting AprilTags.
     */
    private void handleFramerateChanges() {
        if (dashboardCameraFramerate != lastDashboardCameraFramerate) {
            dashboard.stopCameraStream();
            dashboard.startCameraStream(visionPortal, dashboardCameraFramerate);
            lastDashboardCameraFramerate = dashboardCameraFramerate;
        }
    }

    /**
     * Read this camera's minimum and maximum Exposure and Gain settings.
     * Can only be called AFTER calling initializeCamera();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }

}
