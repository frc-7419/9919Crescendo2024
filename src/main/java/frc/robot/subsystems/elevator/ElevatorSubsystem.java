// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX motorOne; 
  private final TalonFX motorTwo;
  private final MotionMagicVoltage mmVoltage;

  public ElevatorSubsystem() {
    this.motorOne = new TalonFX(0);
    this.motorTwo = new TalonFX(0);
    this.mmVoltage = new MotionMagicVoltage(0).withSlot(0);
    motorOne.setInverted(true); //TODO: when robot is built and motor are in place check if inversion is needed
    motorTwo.setInverted(false);//TODO: when robot is built and motor are in place check if inversion is needed
    TalonFXConfiguration config = new TalonFXConfiguration();
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    config.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward                    TODO: Friction, tune kS
    config.Slot0.kV = 0.11299435; // volts per rotation per second                                        TODO: Velocity, tune kV
    config.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 TODO: tune P
    config.Slot0.kI = 0; // No output for integrated error                                                TODO: tune I
    config.Slot0.kD = 0; // No output for error derivative                                                TODO: tune D
    config.Slot0.kG = 0; // Gravity                                                                       TODO: figure out gravity

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfig = config.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 80; // rps cruise velocity TODO
    motionMagicConfig.MotionMagicAcceleration = 160; // rps/s acceleration TODO
    motionMagicConfig.MotionMagicJerk = 1600; // rps/s^2 jerk TODO
    motorOne.getConfigurator().apply(config);
    motorTwo.getConfigurator().apply(config);
    motorTwo.setControl(new Follower(motorOne.getDeviceID(), false));
  }

  /**
   * position is in rotations right now
   * @param position
   */
  public void goToPosition(double position) {
    motorOne.setControl(mmVoltage.withPosition(position));
  }

  @Override
  public void periodic() {
    
  }
}
