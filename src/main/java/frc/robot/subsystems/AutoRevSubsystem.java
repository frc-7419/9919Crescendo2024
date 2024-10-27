// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.concurrent.TimeUnit;

public class AutoRevSubsystem extends SubsystemBase {
  private final ShooterSubsystem shooterSubsystem;
  private final CommandXboxController commandXboxController;
  private final CommandSwerveDrivetrain commandSwerveDrivetrain;
  private boolean inRange;
  private boolean toggleAutoRev;
  private final double lineXValue;
  private double revingTime;
  public AutoRevSubsystem(ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain commandSwerveDrivetrain, CommandXboxController commandXboxController) {
    this.shooterSubsystem = shooterSubsystem;
    this.commandXboxController = commandXboxController;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.revingTime = 0;
    lineXValue = 3; //change after getting actual start line x
    inRange = true;
    toggleAutoRev = true;
  }
  public void revWhenInRange(){
     Pose2d currentPos = commandSwerveDrivetrain.getPose2d();
    double currentX = currentPos.getX();
    if(currentX <= lineXValue){
        inRange = true;
        if (toggleAutoRev) {
          shooterSubsystem.run(Constants.ShooterConstants.topShooterRPM,Constants.ShooterConstants.bottomShooterRPM);
          revingTime += 1;
          TimeUnit.SECONDS.sleep(0);
          revingTime += 1;
          TimeUnit.SECONDS.sleep(0);
          revingTime += 1;
          //the value of reving time is the amount of seconds since the shooter has started revving (someone please look into these syntax errors i need to sleep)



        }
    }else{
        inRange = false;
    }

    if (commandXboxController.getLeftTriggerAxis()>=0.1 && commandXboxController.getRightTriggerAxis()>=0.1) {
      toggleAutoRev = !toggleAutoRev;
      if (!toggleAutoRev) shooterSubsystem.brake();
    }
    
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("In Shoot Distance", inRange);
  }
  public double getRevingTime(){
    return revingTime;

  }
  public boolean getInRange(){
    Pose2d currentPos = commandSwerveDrivetrain.getPose2d();
    double currentX = currentPos.getX();
    if(currentX <= lineXValue){
      return true;

     }
  }
}
