package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

// FTC DASHBOARD IMPORTS
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// VISION ANALYSIS IMPORTS
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
@Config
public class Robot_2 extends LinearOpMode {
    // FTC DASHBOARD
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // OTHER STUFF
    private IMU imu;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor shooterMotorLeft, shooterMotorRight;
    private DcMotor intakeMotor;
    private CRServo leftIntakeServo, rightIntakeServo, middleIntakeServo;
    private double forward, strafe, turn;
    public static double moveSpeedMultiplier = 0.75;
    private double moveDirection = 1.0;
    private double rampPower, shooterPower;
    public static double Shooter = 1950;
    private double shooterEnable = 1.0;

    public static double driveSpeedMultiplier = 1.0;
    private double intakeDirection = -1.0;
    private double intakeServoPower = 0;


    // STATIC VALUES FOR MATH
    private final static double intakeGearRatio = 3.0 / 20.0; // 20:1 gearbox, 1:3 bevel gear
    public static double MAX_INTAKE_RPM = 6000.0 * intakeGearRatio;
    public static double MAX_SHOOTER_RPM = 4200.0;
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

    // CAMERA
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup
        initializeMotors();
        initalizeServos();
        initializeIMU();
        initializeCamera();
        waitForStart();

        // Stop
        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
            pollController();
            driveMotors();
            driveServos();

            runEtc();

            measureRPMs();

            reportTelemetry();
        }
    }

    @SuppressLint("DefaultLocale")
    private void reportTelemetry() {
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

        // CLEANUP
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    private void driveServos() {
        leftIntakeServo.setPower(intakeServoPower);
        rightIntakeServo.setPower(intakeServoPower);
        middleIntakeServo.setPower(-intakeServoPower);
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

    /**
     * Sets up the motors, based on the names given to it in the phone-side config
     */
    private void initializeMotors() {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        backLeftMotor = hardwareMap.dcMotor.get("back_left");
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        backRightMotor = hardwareMap.dcMotor.get("back_right");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotorLeft = hardwareMap.dcMotor.get("shooter_left");
        shooterMotorRight = hardwareMap.dcMotor.get("shooter_right");

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.dcMotor.get("intake");
    }

    private void initalizeServos() {
        middleIntakeServo = hardwareMap.crservo.get("feeder_mid");
        leftIntakeServo = hardwareMap.crservo.get("feeder_left");
        rightIntakeServo = hardwareMap.crservo.get("feeder_right");
    }

    private void pollController() {// Mecanum drive motors
        // INTAKE SERVO DIRECTION
        if (gamepad1.a) {
            intakeServoPower = 1.0;
        } else if (gamepad1.b) {
            intakeServoPower = -1.0;
        } else {
            intakeServoPower = 0.0;
        }

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

        double y = -gamepad1.left_stick_y; // Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        turn = turn * moveSpeedMultiplier;
        forward = forward * moveSpeedMultiplier * moveDirection;
        strafe = strafe * moveSpeedMultiplier * moveDirection;
        ((DcMotorEx) shooterMotorLeft).setVelocity((Shooter / 2.1429) * shooterEnable);
        ((DcMotorEx) shooterMotorRight).setVelocity((Shooter / 2.1429) * shooterEnable);
    }

    private void driveMotors() {
        frontLeftMotor.setPower(forward + strafe + turn);
        frontRightMotor.setPower((forward - strafe) - turn);
        backLeftMotor.setPower((forward - strafe) + turn);
        backRightMotor.setPower((forward + strafe) - turn);
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
    }


    private void measureRPMs() {
        RPMTime = System.currentTimeMillis();
        timeDelta = RPMTime - lastRPMTime;
        if (timeDelta > RPMRefreshTime) {
            int leftShooterPosition = shooterMotorLeft.getCurrentPosition();
            int rightShooterPosition = shooterMotorRight.getCurrentPosition();
            int intakePosition = intakeMotor.getCurrentPosition();

            leftShooterRPM = ((double) (leftShooterPosition - lastLeftShooterPosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Tetrix;
            rightShooterRPM = ((double) (rightShooterPosition - lastRightShooterPosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Tetrix;
            intakeRPM = ((double) (intakePosition - lastIntakePosition)) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution_Rev;

            lastLeftShooterPosition = leftShooterPosition;
            lastRightShooterPosition = rightShooterPosition;
            lastIntakePosition = intakePosition;
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

    }

}