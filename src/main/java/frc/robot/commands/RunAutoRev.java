// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoRevSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.concurrent.TimeUnit;
public class RunAutoRev extends Command {
  private final AutoRevSubsystem revCheckerSubsystem;
  private double revingTime;
  
  public RunAutoRev(AutoRevSubsystem revCheckerSubsystem) {
    this.revCheckerSubsystem = revCheckerSubsystem;
    addRequirements(revCheckerSubsystem);
    this.revingTime = () -> (revCheckerSubsystem.getRevingTime());//constantly updatin rev time value(once again syntax errors please save me)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    () -> revCheckerSubsystem.revWhenInRange();
    if (revingTime == 3 && revCheckerSubsystem.getInRange()){
      //insert firing button code here, i lowkey lost my flowstate
      SmartDashboard.putBoolean(getName("Speaker in range"), isScheduled("is true")); //im not familiar with all the smartdashboard put methods i know this is wrong.

      

    }

    //make this a lambda function so it keeps checking, can someone check out this syntax error?
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public void fire(){
   

  }
}
