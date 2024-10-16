// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.handoff.HandoffSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNote extends SequentialCommandGroup {
    /**
     * Creates a new AutoShoot.
     */
    public OneNote(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, HandoffSubsystem handoffSubsystem, SwerveDriveFieldCentric drivetrain) {
        addCommands(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(2),
                    new RunCommand(() -> {
                        shooterSubsystem.run(3500, 3500);
                    }, shooterSubsystem)
                ),
                new ShootNote(shooterSubsystem, handoffSubsystem, intakeSubsystem)
            )
        );
    }
}