// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeWristSubsystem;

public class IntakeNote extends Command {
  /** Creates a new Auto. */
  private final IntakeSubsystem intake;
  private final IntakeWristSubsystem wrist;
  public IntakeNote(IntakeSubsystem intake, IntakeWristSubsystem wrist) {
    this.intake = intake;
    this.wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.goToPosition(0.9); // Arbitrary value
    intake.run(0.8, 0.8); // Arbitrary value
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}