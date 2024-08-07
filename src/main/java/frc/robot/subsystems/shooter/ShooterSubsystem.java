// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  // Things the subsystem will do:
  // - We want everything in meters, no inches or feet so a conversion ratio is needed. For example, the speed of the motors should be in m/s
  private final TalonFX topMotor; 
  private final TalonFX bottomMotor;
  private final VoltageOut voltageOut;



  public ShooterSubsystem() {
    this.topMotor = new TalonFX(Constants.ShooterConstants.topShooterID,"rio");
    this.bottomMotor = new TalonFX(Constants.ShooterConstants.bottomShooterID, "rio");
    topMotor.setInverted(true);
    this.voltageOut = new VoltageOut(0);
    voltageOut.EnableFOC = true;

    TalonFXConfiguration config = new TalonFXConfiguration();
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    config.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward                    DONT TOUCH
    config.Slot0.kV = 0.11299435; // volts per rotation per second                                        DONT TOUCH
    config.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 TOUCH
    config.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
    config.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
    config.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;

    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);
  }

  /**
   * Runs the motors of the shooter at the given speeds in m/s
   * @param bottomSpeed speed of the bottom motor in m/s, positive will push the note forward 
   * @param topSpeed speed of the top motor in m/s, positive will push the note forward
   */
  public void run(final double topVoltage, final double bottomVoltage) {
    voltageOut.Output = topVoltage;
    topMotor.setControl(voltageOut);
    voltageOut.Output = bottomVoltage;
    bottomMotor.setControl(voltageOut);
  }

  /**
   * Gets the velocity of the top motor
   * @return velocity in m/s positive means forward
   */
  public double getTopVelocity() {
    return topMotor.getVelocity().getValue();
  }

  /**
   * Gets the velocity of the bottom motor
   * @return velocity in m/s positive means forward
   */
  public double getBottomVelocity() {
    return bottomMotor.getVelocity().getValue();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Shooter Velocity", getTopVelocity());
    SmartDashboard.putNumber("Bottom Shooter Velocity", getBottomVelocity());
    SmartDashboard.putNumber("Top Shooter Voltage: ", topMotor.getMotorVoltage().getValue());
    SmartDashboard.putNumber("Bottom Shooter Voltage: ", bottomMotor.getMotorVoltage().getValue());
    SmartDashboard.putNumber("Top Shooter Temperature ", topMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber("Bottom Shooter Temperature", bottomMotor.getDeviceTemp().getValue());
  }
  public void coast() {
    topMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void brake() {
    topMotor.setNeutralMode(NeutralModeValue.Brake);
    bottomMotor.setNeutralMode(NeutralModeValue.Brake);
  }
}
