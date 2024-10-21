// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;

//WORK IN PROGRESS
//Note: Sequential command group is being used with RunShooterCommand because other commands will be used in the future
public class ShootNote extends SequentialCommandGroup {
    public ShootNote(ShooterSubsystem shooterSubsystem) {
        addCommands(
                new RunShooterCommand(shooterSubsystem, 0.9) // Run shooter at 90% power
        );
    }
}