// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

/**
 * Handles the intake process for detecting notes.
 * 
 * Monitors the intake current to detect notes, adjusts the
 * operation of the intake subsystem, and completes once the note
 * is processed or a timeout occurs.
 * 
 */
public class IntakeNote extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final Timer thresholdTimer;
  private final Timer timeoutTimer;
  private final Timer intakeVerificationTimer;
  private boolean init;
  private int notePhase = 0;
  private boolean done;

  /**
   * Constructs an IntakeNote command.
   * 
   * @param intakeSubsystem  the intake subsystem used by this command
   */
  public IntakeNote(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.thresholdTimer = new Timer();
    this.timeoutTimer = new Timer();
    this.intakeVerificationTimer = new Timer();
    addRequirements(intakeSubsystem);
  }

  /**
   * Initializes the command, setting subsystem modes and timers for detecting
   * notes.
   */
  @Override
  public void initialize() {
    intakeSubsystem.coast();
    notePhase = 0;
    done = false;
    thresholdTimer.reset();
    thresholdTimer.start();
    timeoutTimer.reset();
    intakeVerificationTimer.reset();
    init = false;
  }

  /**
   * Main runtime, running the intake subsystem and
   * monitoring for note detection based on current draw.
   */
  @Override
  public void execute() {
    SmartDashboard.putNumber("Note Phase", notePhase);
    if(!init) intakeSubsystem.run(IntakeConstants.INTAKE_POWER);
    // Update baseline current draw after 0.5 seconds
    if (thresholdTimer.hasElapsed(1) && !init) init = true;
    // Detect the note in intake
    if (intakeSubsystem.noteDetectedByCurrent() && init && notePhase == 0) {
      timeoutTimer.start();
      notePhase = 2; // Update to the next phase where handoff takes control
    }

    // After intake gets the note, push it back slightly to verify its position
    if (notePhase == 2 && timeoutTimer.hasElapsed(0.3)) {
      intakeSubsystem.run(-0.3); // Reverse the intake briefly
      intakeVerificationTimer.start();
      if (intakeVerificationTimer.hasElapsed(0.2)) { // Reverse for 0.2 seconds
        intakeSubsystem.run(0); // Stop intake after the pushback
        notePhase = 3;
        done = true; // Note is processed
        intakeVerificationTimer.stop();
      }
    }
  }

  /**
   * Ends the command by stopping both subsystems and resetting timers.
   * 
   * @param interrupted whether the command was interrupted before finishing
   */
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.run(0);
    intakeSubsystem.brake();
    thresholdTimer.stop();
    timeoutTimer.stop();
    intakeVerificationTimer.stop();
  }

  /**
   * Determines whether the command has finished.
   * 
   * @return true if the note has been processed or the command times out
   */
  @Override
  public boolean isFinished() {
    return done || timeoutTimer.hasElapsed(IntakeConstants.MAX_INTAKE_TIME);
  }
}