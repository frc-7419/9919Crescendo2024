// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    /**
     * Constructs a new IntakeSubsystem.
     * Initializes the top and bottom motors, sets their inversion, and configures the Talon FX motors.
     */
    public IntakeSubsystem() {
        // Initialize the top and bottom motors
        this.motor = new CANSparkMax(Constants.IntakeConstants.intakeID, MotorType.kBrushless);
        this.coast();
    }

    /**
     * Runs the intake motors at the specified RPM.
     * If either motor is running, the motors are set to coast mode.
     * If both motors are stopped, the motors are set to brake mode.
     *
     * @param percent    the desired percent for motor
     */
    public void run(final double percent) {
        // If either motor is running, set the motors to coast mode
        if (percent != 0) {
            coast();
            motor.set(percent);
        }
        // If both motors are stopped, set the motors to brake mode
        else {
            brake();
            motor.set(percent);
        }
    }

    /**
     * Gets the current velocity of the top motor.
     *
     * @return the current velocity of the top motor in rotations per second
     */
    public double getTopVelocity() {
        return 0;
    }

    /**
     * Sets the motors to coast mode.
     */
    public void coast() {
        motor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the motors to brake mode.
     */
    public void brake() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Periodic function called by the scheduler.
     * Puts the top and bottom motor velocities on the SmartDashboard.
     */
    @Override
    public void periodic() {
        // Put the top and bottom motor velocities on the SmartDashboard
        SmartDashboard.putNumber("Shooter Top Velocity", getTopVelocity());
    }
}