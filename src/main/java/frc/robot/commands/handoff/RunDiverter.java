// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.handoff;

import edu.wpi.first.wpilibj2.command.Command;

public class RunDiverter extends Command {
  /** Creates a new RunDiverter. */

  // Just apply a given power to the diverter wheel motor.

  // Example call: operator.y().whileTrue(new RunDiverter(diverter, 0.5));

  // Declare the subsystem and variables

  public RunDiverter() {
    // Initialize the subsystem and variables

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize the motor
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run the motor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop when condition is met such as a beam break sensor. Not sure what will be
    // used but use a beam break sensor as a placeholder.
    return false; 
  }
}
