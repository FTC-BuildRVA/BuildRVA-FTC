package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double setpoint = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    private double maxIntegral = 1.0;
    private double outputMin = -1.0;
    private double outputMax = 1.0;

    private boolean angleWrap = false;

    /**
     * Constructor with PID gains
     */
    public PIDController(double kp, double ki, double kd) {
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    /**
     * Set PID gains
     */
    public void setGains(double kp, double ki, double kd) {
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    /**
     * Set the target setpoint
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Get the current setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Enable angle wrapping for angular control (handles -180 to 180 wraparound)
     */
    public void setAngleWrap(boolean enable) {
        this.angleWrap = enable;
    }

    /**
     * Set integral limits to prevent windup
     */
    public void setIntegralLimits(double maxIntegral) {
        this.maxIntegral = Math.abs(maxIntegral);
    }

    /**
     * Set output limits
     */
    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * Calculate PID output
     * @param currentValue The current process value
     * @return The control output
     */
    public double calculate(double currentValue) {
        // Calculate error
        double error = setpoint - currentValue;

        // Handle angle wrapping if enabled
        if (angleWrap) {
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
        }

        // Proportional term
        double p = Kp * error;

        // Integral term with anti-windup
        integralSum += error;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        double i = Ki * integralSum;

        // Derivative term
        double derivative = error - lastError;
        double d = Kd * derivative;
        lastError = error;

        // Calculate total output
        double output = p + i + d;

        // Clamp output
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

    /**
     * Get the current error
     */
    public double getError(double currentValue) {
        double error = setpoint - currentValue;

        if (angleWrap) {
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
        }

        return error;
    }

    /**
     * Reset the controller (clears integral sum and last error)
     */
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
    }

    /**
     * Check if the controller is at setpoint within tolerance
     */
    public boolean atSetpoint(double currentValue, double tolerance) {
        return Math.abs(getError(currentValue)) <= tolerance;
    }
}