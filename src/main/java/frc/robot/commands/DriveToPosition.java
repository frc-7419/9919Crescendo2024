// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class DriveToPosition extends Command {
  /** Creates a new DriveToPosition. */
  private DriveBaseSubsystem driveBaseSubsystem;
  private double vx;
  private double vy;
  private double rx;
  private boolean slowMode;
  public DriveToPosition(DriveBaseSubsystem driveBaseSubsystem, double vx, double vy, double rx, boolean slowMode) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.vx = vx;
    this.vy = vy;
    this.rx = rx;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    driveBaseSubsystem.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBaseSubsystem.setModuleStates(driveBaseSubsystem.getChassisSpeedsFromJoystick(vx, vy, rx, slowMode));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
