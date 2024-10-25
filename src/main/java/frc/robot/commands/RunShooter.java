// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final double topRPM;
  private final double bottomRPM;
  /** Creates a new RunShooter. */
  public RunShooter(ShooterSubsystem shooterSubsystem, double topRPM, double bottomRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.topRPM = topRPM;
    this.bottomRPM = bottomRPM;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.run(topRPM, bottomRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.run(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
