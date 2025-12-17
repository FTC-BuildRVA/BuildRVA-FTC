package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

// VISION ANALYSIS IMPORTS
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// FTC DASHBOARD IMPORTS
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.List;


@TeleOp
@Config
public class ExampleRobot extends LinearOpMode {
    private IMU imu;

    // Use the new MecanumDrive class instead of individual motor variables
    private Utiltiy_MecanumDrive drive;

    private DcMotor intakeMotor;
    private DcMotor shooterMotorLeft;
    private DcMotor shooterMotorRight;
    private DcMotor testMotor;

    TelemetryPacket packet = new TelemetryPacket();

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;

    public static int cameraFramerate = 0;
    private int lastFramerate = cameraFramerate;

    private double intakeDirection = -1.0;

    // STATIC VALUES FOR MATH
    private final static double intakeGearRatio = 3.0 / 20.0; // 20:1 gearbox, 1:3 bevel gear
    public static double MAX_INTAKE_RPM = 6000.0 * intakeGearRatio;
    public static double MAX_SHOOTER_RPM = 5000.0;
    private final double encoderPulsesPerRevolution_Rev = 28.0; // 28 signal edges per revolution encoder
    private final double encoderPulsesPerRevolution_Tetrix = 24.0; // 28 signal edges per revolution encoder

    // VALUES FOR RPM TRACKING
    private double timeDelta = 0;
    public static double targetShooterRPM = 0;
    public static double targetIntakeRPM = 0;
    public static double targetTestMotorPower = 0;
    private int lastIntakePosition = 0;
    private int lastLeftShooterPosition = 0;
    private int lastRightShooterPosition = 0;
    private int lastTestMotorPosition = 0;
    private double RPMTime = System.currentTimeMillis();
    private double lastRPMTime = RPMTime;
    private double RPMRefreshTime = 100; //milliseconds

    // FINAL CALCULATED RPM VALUES
    private double intakeRPM = 0;
    private double leftShooterRPM = 0;
    private double rightShooterRPM = 0;
    private double testMotorRPM = 0;

    // RPM WINDOW FILTER
    private int numberOfAverages = 100;
    private double[] listLeftShooterRPM = new double[numberOfAverages];
    private double[] listRightShooterRPM = new double[numberOfAverages];
    private double[] listRPMCycleTimes = new double[numberOfAverages];
    private int RPMCurrentIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // SETUP
        drive = new Utiltiy_MecanumDrive(hardwareMap);  // Initialize the drive system
        initializeIMU();
        initializeTelemetry();
        initializeCamera();
        initializeEtc();
        waitForStart();

        // ABORT
        if (isStopRequested()) return;

        // MAIN LOOP
        while (opModeIsActive()) {
            // CALCULATE USER INPUTS AND UPDATE DRIVETRAIN

            drive.calculateUserDriveInput(gamepad1);
            drive.update();

            manageFramerateChanges();

            runEtc();
            measureRPMs();

            packet.put("zzz_sanity_check_test", testMotor.getCurrentPosition());
            packet.put("zzz_sanity_check_left", shooterMotorLeft.getCurrentPosition());
            packet.put("zzz_sanity_check_right", shooterMotorRight.getCurrentPosition());

            // WRAP-UP AND TELEMETRY
            reportTelemetry();
        }
    }

    private void initializeEtc() {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        shooterMotorRight = hardwareMap.dcMotor.get("shooter_right");
        shooterMotorLeft = hardwareMap.dcMotor.get("shooter_left");
        testMotor = hardwareMap.dcMotor.get("test_motor");
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void runEtc() {
        // INTAKE DIRECTION
        if (gamepad1.dpad_up) {
            intakeDirection = -1.0;
        }
        if (gamepad1.dpad_down) {
            intakeDirection = 1.0;
        }

        // INTAKE POWER
        if (gamepad1.left_bumper) {
            targetIntakeRPM = gamepad1.left_trigger * MAX_INTAKE_RPM;
        }
        intakeMotor.setPower(targetIntakeRPM / MAX_INTAKE_RPM * intakeDirection);

        // SHOOTER POWER
        if (gamepad1.right_bumper) {
            targetShooterRPM = gamepad1.right_trigger * MAX_SHOOTER_RPM;
        }
        shooterMotorLeft.setPower(targetShooterRPM / MAX_SHOOTER_RPM);
        shooterMotorRight.setPower(targetShooterRPM / MAX_SHOOTER_RPM);

        testMotor.setPower(targetTestMotorPower);
    }

    private void measureRPMs() {
        RPMTime = System.currentTimeMillis();
        timeDelta = RPMTime - lastRPMTime;
        if (timeDelta > RPMRefreshTime) {
            int leftShooterPosition = shooterMotorLeft.getCurrentPosition();
            int rightShooterPosition = shooterMotorRight.getCurrentPosition();
            int intakePosition = intakeMotor.getCurrentPosition();
            int testMotorPosition = testMotor.getCurrentPosition();

            leftShooterRPM = ((double) (leftShooterPosition - lastLeftShooterPosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Tetrix;
            rightShooterRPM = ((double) (rightShooterPosition - lastRightShooterPosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Tetrix;
            intakeRPM = ((double) (intakePosition - lastIntakePosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Rev;
            rightShooterRPM = ((double) (testMotorPosition - lastTestMotorPosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Tetrix;

            lastLeftShooterPosition = leftShooterPosition;
            lastRightShooterPosition = rightShooterPosition;
            lastIntakePosition = intakePosition;
            lastTestMotorPosition = testMotorPosition;
            lastRPMTime = RPMTime;

            listLeftShooterRPM[RPMCurrentIndex] = leftShooterRPM;
            listRightShooterRPM[RPMCurrentIndex] = rightShooterRPM;
            listRPMCycleTimes[RPMCurrentIndex] = timeDelta;

            RPMCurrentIndex++;
            if (RPMCurrentIndex == numberOfAverages) {
                RPMCurrentIndex = 0;
            }
        }
    }

    /**
     * Starts up the IMU (inertial measurement unit, gives us the angle of the robot)
     */
    private void initializeIMU() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    private void initializeCamera() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal using the camera and processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        int minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
//        exposureControl.setExposure((long) minExposure, TimeUnit.MILLISECONDS);
//
//        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//        int maxGain = gainControl.getMaxGain();
//        gainControl.setGain(maxGain);

        dashboard.startCameraStream(visionPortal, cameraFramerate);
    }

    private void initializeTelemetry() {
        dashboard = FtcDashboard.getInstance();
    }

    @SuppressLint("DefaultLocale")
    private void reportTelemetry() {
        // MAKE A NEW PACKET FOR US TO FILL
        Canvas fieldOverlay = packet.fieldOverlay();

        // VARIABLES AND MATH
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // PACK IN THE VARIABLES
        packet.put("Runtime", System.currentTimeMillis());
        packet.put("Battery Voltage", String.format("%.3f V", voltage));
        packet.put("Target intake", String.format("%.3f RPM", targetIntakeRPM));
        packet.put("Target shooter", String.format("%.3f RPM", targetShooterRPM));
        packet.put("RPM intake", String.format("%.3f RPM", intakeRPM));
        packet.put("RPM shooter left", String.format("%.3f RPM", leftShooterRPM));
        packet.put("RPM shooter right", String.format("%.3f RPM", rightShooterRPM));
        packet.put("RPM test motor", String.format("%.3f RPM", testMotorRPM));

        // AVGERAGE THE RPM
        double avgRPMLeft = 0;
        double avgRPMRight = 0;
        double avgRPMCycleTime = 0;
        for (int i = 0; i < numberOfAverages; i++) {
            avgRPMLeft += listLeftShooterRPM[i];
            avgRPMRight += listRightShooterRPM[i];
            avgRPMCycleTime += listRPMCycleTimes[i] / 1000.0;
        }
        avgRPMLeft /= numberOfAverages;
        avgRPMRight /= numberOfAverages;

        packet.put("Measurement RPM cycle time", String.format("%.3f ms", timeDelta));
        packet.put("Window filter width", String.format("%.3f seconds", avgRPMCycleTime));
        packet.put("Average left shooter RPM", String.format("%.3f RPM", avgRPMLeft));
        packet.put("Average right shooter RPM", String.format("%.3f RPM", avgRPMRight));
        packet.put("Graph max", 5000);
        packet.put("Graph min", 0);

        // PACK IN THE FIELD GRAPHICS
        // Draw robot as rotated rectangle
        double robotX = 10;
        double robotY = 20;
        double robotHeading = Math.toRadians(45);
        double robotWidth = 18;  // inches
        double robotLength = 18; // inches

        double halfWidth = robotWidth / 2;
        double halfLength = robotLength / 2;

        // Calculate the four corners
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double[] xPoints = {
                robotX + (-halfLength * cos - -halfWidth * sin),
                robotX + (halfLength * cos - -halfWidth * sin),
                robotX + (halfLength * cos - halfWidth * sin),
                robotX + (-halfLength * cos - halfWidth * sin)
        };

        double[] yPoints = {
                robotY + (-halfLength * sin + -halfWidth * cos),
                robotY + (halfLength * sin + -halfWidth * cos),
                robotY + (halfLength * sin + halfWidth * cos),
                robotY + (-halfLength * sin + halfWidth * cos)
        };

        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.strokePolygon(xPoints, yPoints);

        // Optional: draw heading indicator line
        fieldOverlay.strokeLine(robotX, robotY,
                robotX + halfLength * cos,
                robotY + halfLength * sin);

        // Add april tag telemetry
        telemetryAprilTag(packet);

        // CLEANUP
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag(TelemetryPacket packet) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Also send to FTC Dashboard
        packet.put("# AprilTags Detected", currentDetections.size());

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

                packet.addLine(tagInfo);
                packet.addLine(xyzInfo);
                packet.addLine(pryInfo);
                packet.addLine(rbeInfo);

                dashboard.getTelemetry().addLine(tagInfo);
                dashboard.getTelemetry().addLine(xyzInfo);
                dashboard.getTelemetry().addLine(pryInfo);
                dashboard.getTelemetry().addLine(rbeInfo);
            } else {
                String unknownTag = String.format("\n==== (ID %d) Unknown", detection.id);
                String centerInfo = String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y);

                packet.addLine(unknownTag);
                packet.addLine(centerInfo);

                dashboard.getTelemetry().addLine(unknownTag);
                dashboard.getTelemetry().addLine(centerInfo);
            }
        }
    }

    private void manageFramerateChanges() {
        if (cameraFramerate != lastFramerate) {
            dashboard.stopCameraStream();
            dashboard.startCameraStream(visionPortal, cameraFramerate);
            lastFramerate = cameraFramerate;
        }
    }
}