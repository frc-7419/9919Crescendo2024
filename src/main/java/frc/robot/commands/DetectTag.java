// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.LimelightRangeChecker;

public class DetectTag extends Command {
  private final LimelightRangeChecker limelightRangeChecker;
  private boolean speakerInRange = false;
  public DetectTag(LimelightRangeChecker limelightRangeChecker) {
    this.limelightRangeChecker = limelightRangeChecker;
    addRequirements(limelightRangeChecker);
  }


  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speakerInRange = limelightRangeChecker.speakerFiducialInRange(3); //this still needs testing to see what shooting range is
    SmartDashboard.putBoolean("Speaker in range: ", speakerInRange);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
