// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TurnToSpeaker extends Command {
  DriveBaseSubsystem driveBaseSubsystem;
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  PIDController pidController;
  /** Creates a new TurnToSpeaker. */
  public TurnToSpeaker(DriveBaseSubsystem driveBaseSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    pidController = new PIDController(kP,kI,kD);
  
    addRequirements(driveBaseSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = LimelightHelpers.getTX("limelight"); 
    double rotationSpeed = -pidController.calculate(tx,0); 
 
    rotationSpeed = Math.max(-0.5, Math.min(0.5,rotationSpeed)); // clamp

    driveBaseSubsystem.setModuleStates(
      driveBaseSubsystem.getChassisSpeedsFromJoystick(0,0,rotationSpeed,false)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() && LimelightHelpers.getTV("limelight");
  
  }
}