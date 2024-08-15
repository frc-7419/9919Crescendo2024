// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandoffConstants;

public class HandoffSubsystem extends SubsystemBase {
    final VoltageOut m_request = new VoltageOut(0);
    /**
     * Creates a new HandoffSubsystem.
     */

    /*
     * Just a diverter wheel powered by a falcon.
     *
     * Clockwise: To Elevator
     * Counter-Clockwise: To Intake
     */

    private TalonFX handoffMotor1;
    private TalonFX handoffMotor2;

    public HandoffSubsystem() {
        handoffMotor1 = new TalonFX(HandoffConstants.handoffMotor1ID);
        handoffMotor2 = new TalonFX(HandoffConstants.handoffMotor2ID);
        handoffMotor2.setInverted(true);
        coast();
    }

    // Method to apply power to the handoff motor for the command.

    // The other basic motor control methods. Eg coast, brake, etc.

    public void coast() {
        handoffMotor1.setNeutralMode(NeutralModeValue.Coast);
        handoffMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brake() {
        handoffMotor1.setNeutralMode(NeutralModeValue.Brake);
        handoffMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void stop() {
        handoffMotor1.set(0);
        handoffMotor2.set(0);
    }

    public void setVoltage(float voltage) {
        handoffMotor1.setControl(m_request.withOutput(voltage));
        handoffMotor2.setControl(m_request.withOutput(voltage));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
