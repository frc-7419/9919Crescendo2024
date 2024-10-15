// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * Creates a new IntakeSubsystem.
     */

    // Declare the top and bottom motors
    private final TalonFX motor;
    private final VelocityVoltage velocityVoltage;

    /**
     * Constructs a new IntakeSubsystem.
     * Initializes the top and bottom motors, sets their inversion, and configures the Talon FX motors.
     */
    public IntakeSubsystem() {
        // Initialize the top and bottom motors
        this.motor = new TalonFX(Constants.IntakeConstants.intakeID, Constants.RobotConstants.kCanbus);
        this.velocityVoltage = new VelocityVoltage(0).withSlot(0);

        // Invert the top motor, but not the bottom motor
        motor.setInverted(true);

        // Configure the Talon FX motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.1; // To account for friction                                                     TODO: calculate friction
        config.Slot0.kV = 0.11299435; // volts per rotation per second                                        DONT TOUCH
        config.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 TODO: tune value for P
        config.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
        config.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
        config.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
        config.Voltage.PeakForwardVoltage = 12; // Maximum forward voltage
        config.Voltage.PeakReverseVoltage = -12; // Maximum reverse voltage
        motor.getConfigurator().apply(config);
    }

    /**
     * Runs the intake motors at the specified RPM.
     * If either motor is running, the motors are set to coast mode.
     * If both motors are stopped, the motors are set to brake mode.
     *
     * @param RPM    the desired RPM for the top motor
     */
    public void run(final double RPM) {
        // If either motor is running, set the motors to coast mode
        if (RPM != 0) {
            coast();
            motor.setControl(velocityVoltage.withVelocity(RPM / 60.0D));
        }
        // If both motors are stopped, set the motors to brake mode
        else {
            brake();
            motor.setControl(velocityVoltage.withVelocity(RPM / 60.0D));
        }
    }

    /**
     * Gets the current velocity of the top motor.
     *
     * @return the current velocity of the top motor in rotations per second
     */
    public double getTopVelocity() {
        return motor.getVelocity().getValue();
    }

    /**
     * Sets the motors to coast mode.
     */
    private void coast() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets the motors to brake mode.
     */
    private void brake() {
        motor.setNeutralMode(NeutralModeValue.Brake);
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