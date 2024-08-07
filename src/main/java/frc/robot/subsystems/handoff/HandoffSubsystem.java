// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.handoff;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandoffConstants;

public class HandoffSubsystem extends SubsystemBase {
  /** Creates a new HandoffSubsystem. */

  /*
   * Just a diverter wheel powered by a falcon.
   * 
   * Clockwise: To Elevator
   * Counter-Clockwise: To Intake
   */

  private TalonFX handoffMotor;
  final VoltageOut m_request = new VoltageOut(0);

  public HandoffSubsystem() {
    handoffMotor = new TalonFX(HandoffConstants.handoffMotorID);
    coast();
  }

  // Method to apply power to the handoff motor for the command.

  // The other basic motor control methods. Eg coast, brake, etc.

  public void coast() {
    handoffMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    handoffMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void stop() {
    handoffMotor.set(0);
  }

  public void setVoltage(float voltage) {
    // that is not a valid way of setting the speed, either use voltage or velocity voltage. phoenix6 doesn't have percent ouput or an equvilant
    // look at this page you will see what you are trying to do is from v5, right under that is the way to do it in v6 https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/control-requests-guide.html
    handoffMotor.setControl(m_request.withOutput(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
