// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double power;

    public RunShooterCommand(ShooterSubsystem shooterSubsystem, double power) {
        this.shooterSubsystem = shooterSubsystem;
        this.power = power;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
      shooterSubsystem.coast(); // Set the motors to coast mode
        shooterSubsystem.run(power, power); // Assuming both motors run at the same power
    }

    @Override
    public void execute() {
      
    }
    @Override
    public void end(boolean interrupted) {
      if (!DriverStation.isAutonomous()) {
        shooterSubsystem.stop();
        shooterSubsystem.brake(); // Set the motors to brake mode
      }
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}
