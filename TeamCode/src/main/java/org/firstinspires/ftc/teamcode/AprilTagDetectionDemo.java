package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Detection Demo", group = "Vision")
public class AprilTagDetectionDemo extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        initAprilTag();

        // Start streaming camera to FTC Dashboard
        dashboard.startCameraStream(visionPortal, 0);

        // Wait for the DS start button to be touched
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Touch Play to start");
        telemetry.addData("Dashboard", "Camera streaming at http://192.168.43.1:8080/dash");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();

                // Push telemetry to the Driver Station
                telemetry.update();

                // Share the CPU
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal using the camera and processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Display AprilTag detection info on telemetry and camera feed
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Also send to FTC Dashboard
        dashboard.getTelemetry().addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                String tagInfo = String.format("\n==== (ID %d) %s",
                        detection.id, detection.metadata.name);
                String xyzInfo = String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                String pryInfo = String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                String rbeInfo = String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);

                telemetry.addLine(tagInfo);
                telemetry.addLine(xyzInfo);
                telemetry.addLine(pryInfo);
                telemetry.addLine(rbeInfo);

                dashboard.getTelemetry().addLine(tagInfo);
                dashboard.getTelemetry().addLine(xyzInfo);
                dashboard.getTelemetry().addLine(pryInfo);
                dashboard.getTelemetry().addLine(rbeInfo);
            } else {
                String unknownTag = String.format("\n==== (ID %d) Unknown", detection.id);
                String centerInfo = String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y);

                telemetry.addLine(unknownTag);
                telemetry.addLine(centerInfo);

                dashboard.getTelemetry().addLine(unknownTag);
                dashboard.getTelemetry().addLine(centerInfo);
            }
        }

        // Add a blank line between detections
        telemetry.addLine();
        dashboard.getTelemetry().update();
    }
}