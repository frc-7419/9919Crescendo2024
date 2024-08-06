// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Things the subsystem will do:
  // - Spin top and bottom shooter motors(Falcons)
  // - Have a PID system
  // - If possible get a feedforward
  // - Beambreak
  // - The shooter will be at a fixed angle
  // - We want everything in meters, no inches or feet so a conversion ratio is needed. For example, the speed of the motors should be in m/s
  public ShooterSubsystem() {

  }

  /**
   * Runs the motors of the shooter at the given speeds in m/s
   * @param bottomSpeed speed of the bottom motor in m/s, positive will push the note forward 
   * @param topSpeed speed of the top motor in m/s, positive will push the note forward
   */
  public void run(final int bottomSpeed, final int topSpeed) {

  }

  /**
   * Gets the velocity of the top motor
   * @return velocity in m/s positive means forward
   */
  public double getTopVelocity() {
    return 0;
  }

  /**
   * Gets the velocity of the bottom motor
   * @return velocity in m/s positive means forward
   */
  public double getBottomVelocity() {
    return 0;
  }

  @Override
  public void periodic() {
    // Some things to send to smartdashboard periodically
    // - Speed to top and bottom shooter wheels/motors
    // - Voltage of top and bottom shooter wheels/motors if possible
    // - Temprature of top and bottom shooter wheels/motors if possible
    // - Current of top and bottom shooter wheels/motors if possible
  }
}
