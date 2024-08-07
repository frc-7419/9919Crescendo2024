// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;

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

  public HandoffSubsystem() {
    handoffMotor = new TalonFX(HandoffConstants.handoffMotorID);
    coast();
  }

  // Method to apply power to the handoff motor for the command.

  // The other basic motor control methods. Eg coast, brake, etc.

  public void coast() {
    handoffMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void brake() {
    handoffMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void stop() {
    handoffMotor.set(0);
  }

  public void setSpeed(float speed) {
    handoffMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
