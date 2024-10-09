// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.commands.RunDiverter;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.handoff.HandoffSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
    public ShootNote(ShooterSubsystem shooterSubsystem, HandoffSubsystem handoffSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new WaitCommand(1.5),
                                        new RunShooter(shooterSubsystem, 0.9)
                                ),
                                new ParallelDeadlineGroup(
                                        new RunDiverter(handoffSubsystem, (float)0.5).withTimeout(0.5), //arbitrary values
                                        new RunShooter(shooterSubsystem, 0.8)
                                )
                        )
                )
        );
    }
}
       