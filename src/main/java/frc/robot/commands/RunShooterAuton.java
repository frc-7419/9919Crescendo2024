// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunShooterAuton extends SequentialCommandGroup {
        public RunShooterAuton(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
                addCommands(
                                new ParallelRaceGroup(
                                                new WaitCommand(2),
                                                new RunIntake(intakeSubsystem),
                                                new RunShooter(shooterSubsystem, ShooterConstants.topShooterRPM,
                                                                ShooterConstants.bottomShooterRPM)));
        }
}
