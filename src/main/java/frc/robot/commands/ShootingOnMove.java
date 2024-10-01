// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootingOnMove extends Command {
  /** Creates a new ShootingOnMove. */
  DriveBaseSubsystem driveBaseSubsystem;
  ShooterSubsystem shooterSubsystem;
  private final PIDController aimController;
  private final double shooterAngle;
  public final double shooterSpeed;
  private static final double aimTolerance = 1.0; // degrees 
  private static final double maxSpeedAdjustment = 0.2;
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  public Joystick joystick;
  public ShootingOnMove(DriveBaseSubsystem driveBaseSubsystem, ShooterSubsystem shooterSubsystem, Joystick joystick) {
    // initializing subsystems + variables
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterAngle = 60.0; // this is an arbitrary angle in degrees; replace with correct angle value
    this.shooterSpeed = 0.9; 
    this.aimController = new PIDController(kP, kI, kD);
    this.aimController.setTolerance(aimTolerance);
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem, shooterSubsystem);
  }
// gtg
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds currentSpeeds = driveBaseSubsystem.getChassisSpeedsFromJoystick(joystick.getX(), joystick.getY(), joystick.getZ(), false);

    double tx = LimelightHelpers.getTX("limelight"); 
    double rotationAdjustment = aimController.calculate(tx,0); 
    
    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
      currentSpeeds.vxMetersPerSecond,
      currentSpeeds.vyMetersPerSecond,
      currentSpeeds.omegaRadiansPerSecond	+ rotationAdjustment
    );
 
    // rotationAdjustment = Math.max(-0.5, Math.min(0.5,rotationSpeed)); // clamp

    driveBaseSubsystem.setModuleStates(
      driveBaseSubsystem.getChassisSpeedsFromJoystick(adjustedSpeeds.vxMetersPerSecond,adjustedSpeeds.vyMetersPerSecond,adjustedSpeeds.omegaRadiansPerSecond,false)
    );
    
    shooterSubsystem.run(100.0, 100.0); // arbitrary values; should be fixed
    
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
