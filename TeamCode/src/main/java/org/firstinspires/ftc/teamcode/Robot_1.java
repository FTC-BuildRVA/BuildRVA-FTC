package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// 9.5 inches
// 16.25 inches

@TeleOp
@Config
public class Robot_1 extends LinearOpMode {
    private IMU imu;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor shooterMotorLeft, shooterMotorRight;
    private CRServo leftIntakeServo, rightIntakeServo, rampServo1, rampServo2, feederServo1;
    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private double intakePower, rampPower, shooterPower;

    private double timeDelta = 0;
    public static double targetShooterRPM = 0;
    private double leftShooterRPM = 0;
    private double rightShooterRPM = 0;
    private final double encoderPulsesPerRevolution = 28.0; // 28 signal edges per revolution encoder
    private double lastLeftShooterPosition = 0;
    private double lastRightShooterPosition = 0;
    private double RPMTime = System.currentTimeMillis();
    private double lastRPMTime = RPMTime;
    private double RPMRefreshTime = 10; //milliseconds
    private int numberOfAverages = 100;
    private double[] listLeftShooterRPM = new double[numberOfAverages];
    private double[] listRightShooterRPM = new double[numberOfAverages];
    private int RPMCurrentIndex = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double driveSpeedMultiplier = 0.5;

    private int last_encoder_pos_left, last_encoder_pos_right = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup
        initializeMotors();
        initializeIMU();
        waitForStart();

        // Stop
        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
            pollController();
            driveMotors();
            driveServos();
            driveShooter();

            measureRPMs();

            reportTelemetry();
        }
    }

    @SuppressLint("DefaultLocale")
    private void reportTelemetry() {
        // MAKE A NEW PACKET FOR US TO FILL
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // VARIABLES AND MATH
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // PACK IN THE VARIABLES
        packet.put("Battery Voltage", String.format("%.3fV", voltage));
        packet.put("Left shooter RPM", leftShooterRPM);
        packet.put("Right shooter RPM", rightShooterRPM);

        // AVGERAGE THE RPM
        double avgRPMLeft = 0;
        double avgRPMRight = 0;
        for (int i = 0; i < numberOfAverages; i++) {
            avgRPMLeft += listLeftShooterRPM[i];
            avgRPMRight += listRightShooterRPM[i];
        }
        avgRPMLeft /= numberOfAverages;
        avgRPMRight /= numberOfAverages;

        packet.put("RPM refresh frequency", 1.0 / (timeDelta / 1000.0));
        packet.put("Averaging frequency", 1.0 / ((timeDelta / 1000.0) * numberOfAverages));
        packet.put("Average left shooter RPM", avgRPMLeft);
        packet.put("Average right shooter RPM", avgRPMRight);
        packet.put("Graph max", 5000);
        packet.put("Graph min", 0);

        // CLEANUP
        dashboard.sendTelemetryPacket(packet);
    }

    private void driveServos() {
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        rampServo1.setPower(-intakePower);
        rampServo2.setPower(-intakePower);
        feederServo1.setPower(-intakePower);
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

        shooterMotorLeft = hardwareMap.dcMotor.get("shooter_left");
        shooterMotorRight = hardwareMap.dcMotor.get("shooter_right");
        shooterMotorLeft.setMode(RUN_USING_ENCODER);
        shooterMotorRight.setMode(RUN_USING_ENCODER);
        shooterMotorLeft.setDirection(REVERSE);

        leftIntakeServo = hardwareMap.get(CRServo.class, "intake_left");
        rightIntakeServo = hardwareMap.get(CRServo.class, "intake_right");
        rampServo1 = hardwareMap.get(CRServo.class, "ramp_1");
        rampServo2 = hardwareMap.get(CRServo.class, "ramp_2");
        feederServo1 = hardwareMap.get(CRServo.class, "feeder_1");
    }

    private void pollController() {// Mecanum drive motors
        // SERVO POWER
        // Servo motors for the intake and ramp systems
        if (gamepad1.left_bumper) {
            intakePower = -1;
        } else {
            intakePower = gamepad1.left_trigger;
        }
        rampPower = 1;

        // SHOOTER POWER
        if (gamepad1.right_bumper) {
            shooterPower = gamepad1.right_trigger;
        }

        double maxValue = 1;
        int counter = 0;
        if (gamepad1.right_bumper) {
            shooterPower = gamepad1.right_trigger;
            if (gamepad1.a) {
                shooterPower = maxValue * 0.0;
                targetShooterRPM = 0;
                counter += 1;
            }
            if (gamepad1.b) {
                shooterPower = maxValue * 0.25;
                targetShooterRPM = 2500;
                counter += 1;
            }
            if (gamepad1.x) {
                shooterPower = maxValue * 0.50;
                targetShooterRPM = 3000;
                counter += 1;
            }
            if (gamepad1.y) {
                shooterPower = maxValue * 0.75;
                targetShooterRPM = 3500;
                counter += 1;
            }

            if (counter == 4) {
                shooterPower = 1.0;
            }
        }


        // DRIVETRAIN POWER
        double y = -gamepad1.left_stick_y; // Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator * driveSpeedMultiplier;
        backLeftPower = (y - x + rx) / denominator * driveSpeedMultiplier;
        frontRightPower = -(y - x - rx) / denominator * driveSpeedMultiplier;
        backRightPower = -(y + x - rx) / denominator * driveSpeedMultiplier;
    }

    private void driveMotors() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void driveShooter() {
        shooterMotorLeft.setPower(targetShooterRPM / 5000.0);
        shooterMotorRight.setPower(targetShooterRPM / 5000.0);
    }

    private void measureRPMs() {
        RPMTime = System.currentTimeMillis();
        timeDelta = RPMTime - lastRPMTime;
        if (timeDelta > RPMRefreshTime) {
            int leftShooterPosition = shooterMotorLeft.getCurrentPosition();
            int rightShooterPosition = shooterMotorRight.getCurrentPosition();

            leftShooterRPM = (leftShooterPosition - lastLeftShooterPosition) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution;
            rightShooterRPM = (rightShooterPosition - lastRightShooterPosition) / (timeDelta / 1000.0 / 60.0) / encoderPulsesPerRevolution;

            lastLeftShooterPosition = leftShooterPosition;
            lastRightShooterPosition = rightShooterPosition;
            lastRPMTime = RPMTime;

            listLeftShooterRPM[RPMCurrentIndex] = leftShooterRPM;
            listRightShooterRPM[RPMCurrentIndex] = rightShooterRPM;

            RPMCurrentIndex++;
            if (RPMCurrentIndex == numberOfAverages) {
                RPMCurrentIndex = 0;
            }
        }
    }

}