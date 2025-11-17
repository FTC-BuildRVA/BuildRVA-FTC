package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    private double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    public static double driveSpeedMultiplier = 1.0;

    private double forwards = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;

    // PID Controller for heading
    private PIDController headingController;
    private boolean usePIDControl = false;

    /**
     * Constructor - initializes the motors and IMU from the hardware map
     */
    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        backLeftMotor = hardwareMap.dcMotor.get("back_left");
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        backRightMotor = hardwareMap.dcMotor.get("back_right");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize PID controller with default gains
        headingController = new PIDController(0.02, 0.001, 0.0);
        headingController.setAngleWrap(true);
        headingController.setIntegralLimits(0.5);
        headingController.setOutputLimits(-1.0, 1.0);
    }

    /**
     * Calculates mecanum drive powers based on gamepad input
     */
    public void calculateUserDriveInput(Gamepad gamepad) {
        forwards = -gamepad.left_stick_y;
        strafe = gamepad.left_stick_x;
        turn = gamepad.right_stick_x;

        // If user is manually turning, disable PID control
        if (Math.abs(turn) > 0.05) {
            usePIDControl = false;
            headingController.reset();
        }
    }

    /**
     * Enable PID control and set target angle
     *
     * @param angle Target angle in degrees
     */
    public void setTargetAngle(double angle) {
        headingController.setSetpoint(angle);
        usePIDControl = true;
        headingController.reset();
    }

    /**
     * Manually set turn power (disables PID control)
     */
    public void setTurn(double turnPower) {
        turn = turnPower;
        usePIDControl = false;
        headingController.reset();
    }

    /**
     * Set PID controller gains
     */
    public void setHeadingPID(double kp, double ki, double kd) {
        headingController.setGains(kp, ki, kd);
    }

    /**
     * Get the heading PID controller for advanced configuration
     */
    public PIDController getHeadingController() {
        return headingController;
    }

    /**
     * Sets the calculated powers to the motors
     */
    public void update() {
        // Use PID control for turning if enabled
        if (usePIDControl) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            turn = headingController.calculate(currentAngle);
        }

        // Normalize powers so they don't exceed 1.0
        double denominator = Math.max(Math.abs(strafe) + Math.abs(forwards) + Math.abs(turn), 1);

        frontLeftPower = (forwards + strafe + turn) / denominator * driveSpeedMultiplier;
        backLeftPower = (forwards - strafe + turn) / denominator * driveSpeedMultiplier;
        frontRightPower = -(forwards - strafe - turn) / denominator * driveSpeedMultiplier;
        backRightPower = -(forwards + strafe - turn) / denominator * driveSpeedMultiplier;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Stops all drive motors and disables PID control
     */
    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        usePIDControl = false;
        headingController.reset();
    }

    /**
     * Get current heading error (for telemetry)
     */
    public double getHeadingError() {
        if (!usePIDControl) return 0.0;
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return headingController.getError(currentAngle);
    }

    /**
     * Check if robot is at target heading
     */
    public boolean atTargetHeading(double tolerance) {
        if (!usePIDControl) return false;
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return headingController.atSetpoint(currentAngle, tolerance);
    }

    /**
     * Get current motor powers for telemetry
     */
    public double[] getMotorPowers() {
        return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
    }
}