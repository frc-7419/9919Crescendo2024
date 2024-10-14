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

      
    private final TalonFX handoffMotor;

    public HandoffSubsystem() {
        handoffMotor = new TalonFX(HandoffConstants.motorID, Constants.RobotConstants.kCanbus);
        handoffMotor.setInverted(false);
        coast();
    }

    // Method to apply power to the handoff motor for the command.

    // The other basic motor control methods. Eg coast, brake, etc.

    /*Setting the handoff motors to neutral mode. */
    public void coast() {
        handoffMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    /*Setting the handoff motors to brake mode. */
    public void brake() {
        handoffMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    /*Setting the handoff motors to stop. */
    public void stop() {
        handoffMotor.set(0);
    }
/*Outputitng the voltage of the handoff motor*/
    public void setVoltage(final double voltage) {
        handoffMotor.setControl(m_request.withOutput(voltage));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
