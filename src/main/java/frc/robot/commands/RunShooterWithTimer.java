// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class RunShooterWithTimer extends Command {
  private final ShooterSubsystem shooter;
  private final double speed;
  private static final double minimum = 12.7;
  private final Timer timer = new Timer();
  /** Creates a new ShootNote. */
  public RunShooterWithTimer(ShooterSubsystem shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
  //Command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();  }

  // Called every time the scheduler runs while the command is sched/led.
  @Override
  public void execute() {
    timer.start();


    double currentVoltage = RobotController.getBatteryVoltage();


    if (timer.get() <= 0.5) { // Value needs to be changed - arbitrary
        if(currentVoltage < 12.7) {
        double voltageCompensatedSpeed = speed * (currentVoltage / minimum);
    
    // Limit speed to a maximum of 1.0 for safety
    voltageCompensatedSpeed = Math.max(voltageCompensatedSpeed, 1.0);

    
    shooter.run(voltageCompensatedSpeed, voltageCompensatedSpeed);
      }
      else {
        shooter.run(speed, speed);
      }
    }
    //runs if the battery voltage is less than 12.7 to compensate for undervolted motor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}