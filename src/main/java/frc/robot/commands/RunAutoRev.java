// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AutoRevSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.concurrent.TimeUnit;

public class RunAutoRev extends Command {
  private final AutoRevSubsystem revCheckerSubsystem;
  private double revingTime;
  private final ShooterSubsystem shooterSubsystem;
  private final CommandXboxController xboxController;
 
  
  public RunAutoRev(AutoRevSubsystem revCheckerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.revCheckerSubsystem = revCheckerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.xboxController = xboxController;
    addRequirements(revCheckerSubsystem, shooterSubsystem);
    
//constantly updatin rev time value(once again syntax errors please save me)
  }//i don't get this syntax error 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.revingTime = () -> (revCheckerSubsystem.getRevingTime());//aryan  told me to move this here
    () -> revCheckerSubsystem.revWhenInRange();//still dont get this syntax error.
    if (revingTime == 3 && revCheckerSubsystem.getInRange()){
      //insert firing button code here, i lowkey lost my flowstate
      SmartDashboard.putBoolean(getName("Shooter is revved"), isScheduled("is true")); //im not familiar with all the smartdashboard put methods i know this is wrong.
      //still dont know how to code this, this code is probably wrong.
      if (xboxController.leftBumper().getAsBoolean()){
        shooterSubsystem.run(2000, 2000);
      }
      


      

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
