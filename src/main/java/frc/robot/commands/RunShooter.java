// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooter extends Command {
  private final ShooterSubsystem shooter;
  private final double speed;
  /** Creates a new ShootNote. */
  public RunShooter(ShooterSubsystem shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
  //Command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is sched/led.
  @Override
  public void execute() {
    shooter.run(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}