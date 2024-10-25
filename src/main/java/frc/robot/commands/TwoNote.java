// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.SwerveDriveFieldCentric;

public class TwoNote extends SequentialCommandGroup {
  /** Creates a new TwoNote. */
  public TwoNote(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, DriveBaseSubsystem drivetrain) {
    addCommands(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(2),
                    new RunCommand(() -> {
                        shooterSubsystem.run(3500, 3500);

                    }, shooterSubsystem)
                ),
                new DriveToPosition(drivetrain, 0.8, 0, 0, false),
                new IntakeNote(intakeSubsystem),
                new DriveToPosition(drivetrain, -0.8, 0, 0, false),
                new ShootNote(shooterSubsystem, intakeSubsystem)
            )
        );
  }
}
