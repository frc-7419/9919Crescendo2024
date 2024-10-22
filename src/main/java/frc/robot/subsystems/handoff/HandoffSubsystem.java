// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.handoff;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandoffConstants;

public class HandoffSubsystem extends SubsystemBase {
    /*
     * Just a diverter wheel powered by a rev.
     *
     * Clockwise: To Elevator
     * Counter-Clockwise: To Intake
     */

    private final CANSparkMax handoffMotor;
    private double baselineCurrentDraw;

    public HandoffSubsystem() {
        handoffMotor = new CANSparkMax(HandoffConstants.loaderID, MotorType.kBrushless);
        handoffMotor.setInverted(true);
        coast();
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
        double currentDraw = handoffMotor.getOutputCurrent();
        return currentDraw > HandoffConstants.CURRENT_THRESHOLD;
    }

    /**
     * Updates the baseline current draw of the motor.
     */
    public void updateBaselineCurrentDraw() {
        baselineCurrentDraw = handoffMotor.getOutputCurrent();
    }

    /**
     * Gets the current output of the top motor.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double getOutput() {
        return handoffMotor.get();
    }

    // Method to apply power to the handoff motor for the command.

    // The other basic motor control methods. Eg coast, brake, etc.

    /* Setting the handoff motors to neutral mode. */
    public void coast() {
        handoffMotor.setIdleMode(IdleMode.kCoast);
    }

    /* Setting the handoff motors to brake mode. */
    public void brake() {
        handoffMotor.setIdleMode(IdleMode.kCoast);
    }

    /* Setting the handoff motors to stop. */
    public void stop() {
        handoffMotor.set(0);
    }

    /* Outputitng the voltage of the handoff motor */
    public void run(final double percent) {
        // If motor is running, set the motor to coast mode
        if (percent != 0) {
            coast();
            handoffMotor.set(percent);
        }
        // If motor is stopped, set the motor to brake mode
        else {
            brake();
            handoffMotor.set(percent);
        }
    }

    /**
     * Gets the current draw of the motor.
     * 
     * @returns The motor controller's output current in Amps.
     */
    public double getCurrent() {
        return handoffMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Handoff Output", getOutput());
        SmartDashboard.putNumber("Handoff Output Current", handoffMotor.getOutputCurrent());
        SmartDashboard.putNumber("Handoff Baseline Current Draw", baselineCurrentDraw);
    }
}
