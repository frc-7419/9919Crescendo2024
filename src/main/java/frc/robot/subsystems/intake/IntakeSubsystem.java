// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private double baselineCurrentDraw;

    /**
     * Constructs a new IntakeSubsystem.
     * Initializes the top and bottom motors, sets their inversion, and configures
     * the Talon FX motors.
     */
    public IntakeSubsystem() {
        // Initialize the top and bottom motors
        this.intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
        this.coast();
    }

    /**
     * Runs the intake motors at the specified RPM.
     * If either motor is running, the motors are set to coast mode.
     * If both motors are stopped, the motors are set to brake mode.
     *
     * @param percent the desired percent for motor
     */
    public void run(final double percent) {
        // If either motor is running, set the motors to coast mode
        if (percent != 0) {
            coast();
            intakeMotor.set(percent);
        }
        // If both motors are stopped, set the motors to brake mode
        else {
            brake();
            intakeMotor.set(percent);
        }
    }

    /**
     * Gets the current output of the top motor.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double getOutput() {
        return intakeMotor.get();
    }

    /**
     * 
     * Checks if there is a note intaken using voltage compensation.
     * Checks if the current draw of the motor is significantly different from the
     * baseline current draw.
     * 
     * @return true if a note is detected, false otherwise
     */
    public boolean noteDetectedByCurrent() {
        double currentDraw = intakeMotor.getOutputCurrent();
        return Math.abs(currentDraw - baselineCurrentDraw) > IntakeConstants.CURRENT_THRESHOLD;
    }

    /**
     * Updates the baseline current draw of the motor.
     */
    public void updateBaselineCurrentDraw() {
        baselineCurrentDraw = intakeMotor.getOutputCurrent();
    }

    /**
     * Sets the motors to coast mode.
     */
    public void coast() {
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the motors to brake mode.
     */
    public void brake() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Gets the current draw of the motor.
     * 
     * @returns The motor controller's output current in Amps.
     */
    public double getCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    /**
     * Periodic function called by the scheduler.
     * Puts the motor velocities on the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Output", getOutput());
        SmartDashboard.putNumber("Intake Output Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Baseline Current Draw", baselineCurrentDraw);
    }
}