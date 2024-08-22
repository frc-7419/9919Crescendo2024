// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HandoffConstants;

/**
 * The HandoffSubsystem controls the diverter wheel, powered by two Falcon motors.
 * This subsystem handles the transition of game pieces between the intake and elevator systems.
 */
public class HandoffSubsystem extends SubsystemBase {
    final VoltageOut m_request = new VoltageOut(0);

    /**
     * Creates a new HandoffSubsystem.
     */

    /*
     * Just a diverter wheel powered by a Falcon.
     *
     * Clockwise: To Elevator
     * Counter-Clockwise: To Intake
     */

    private final TalonFX handoffMotorOne;
    private final TalonFX handoffMotorTwo;

    /**
     * Constructor for the HandoffSubsystem.
     * Initializes the two Falcon motors and sets their default mode to coast.
     */
    public HandoffSubsystem() {
        handoffMotorOne = new TalonFX(HandoffConstants.motorOneID, Constants.RobotConstants.kCanbus);
        handoffMotorTwo = new TalonFX(HandoffConstants.motorTwoID, Constants.RobotConstants.kCanbus);
        handoffMotorTwo.setInverted(true);  // Inverts the direction of the second motor
        coast();
    }

    /**
     * Sets the motors to coast mode.
     */
    public void coast() {
        handoffMotorOne.setNeutralMode(NeutralModeValue.Coast);
        handoffMotorTwo.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets the motors to brake mode.
     */
    public void brake() {
        handoffMotorOne.setNeutralMode(NeutralModeValue.Brake);
        handoffMotorTwo.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Stops both motors by setting their output to zero.
     */
    public void stop() {
        handoffMotorOne.set(0);
        handoffMotorTwo.set(0);
    }

    /**
     * Sets the output voltage of both motors, controlling the speed and direction of the handoff wheel.
     *
     * @param voltage The desired voltage to apply to the motors.
     */
    public void setVoltage(final double voltage) {
        handoffMotorOne.setControl(m_request.withOutput(voltage));
        handoffMotorTwo.setControl(m_request.withOutput(voltage));
    }

    /**
     * This method is called once per scheduler run to perform periodic actions.
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
