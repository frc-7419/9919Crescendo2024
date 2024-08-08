// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class IntakeWristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final MotionMagicVoltage mmVoltage;

  public IntakeWristSubsystem() {
    this.wristMotor = new TalonFX(0, "rio");
    this.mmVoltage = new MotionMagicVoltage(0).withSlot(0);
    wristMotor.setInverted(false);
    TalonFXConfiguration config = new TalonFXConfiguration();
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    config.Slot0.kS = 0.1; // Friction                                                                    TODO: Friction, tune kS
    config.Slot0.kV = 0.11299435; // volts per rotation per second                                        TODO: tune kV
    config.Slot0.kG = 0; // Gravity                                                                       TODO: figure out gravity
    config.Slot0.kP = 0.11; // Dependent on position                                                      TODO: tune P
    config.Slot0.kI = 0; // Dependent on accumulated error                                                TODO: tune I
    config.Slot0.kD = 0; // Dependent on speed                                                            TODO: tune D

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfig = config.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 80; // rps cruise velocity TODO
    motionMagicConfig.MotionMagicAcceleration = 160; // rps/s acceleration TODO
    motionMagicConfig.MotionMagicJerk = 1600; // rps/s^2 jerk TODO

    // Apply config
    wristMotor.getConfigurator().apply(config);
  }

  /**
   * position is in rotations right now
   * @param position
   */
  public void goToPosition(double position) {
    wristMotor.setControl(mmVoltage.withPosition(position));
  }

  @Override
  public void periodic() {
    
  }
}
