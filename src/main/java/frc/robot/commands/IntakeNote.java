// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handoff.HandoffSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.IntakeConstants;

/**
 * Handles the intake and handoff process for detecting notes
 * 
 * Monitors the intake current to detect notes, adjusts the
 * operation of the intake and handoff subsystems, and completes once the note
 * is processed or a timeout occurs.
 * 
 */
public class IntakeNote extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final HandoffSubsystem handoffSubsystem;
    private final Timer thresholdTimer;
    private final Timer timeoutTimer;
    private final Timer endTimer;
    private boolean init;
    private boolean notePhaseOne;
    private boolean done;

    /**
     * Constructs an IntakeNote command.
     * 
     * @param intakeSubsystem  the intake subsystem used by this command
     * @param handoffSubsystem the handoff subsystem used by this command
     */
    public IntakeNote(IntakeSubsystem intakeSubsystem, HandoffSubsystem handoffSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.handoffSubsystem = handoffSubsystem;
        this.thresholdTimer = new Timer();
        this.timeoutTimer = new Timer();
        this.endTimer = new Timer();
        addRequirements(intakeSubsystem);
    }

    /**
     * Initializes the command, setting subsystem modes and timers for detecting
     * notes.
     */
    @Override
    public void initialize() {
        intakeSubsystem.coast();
        notePhaseOne = false;
        done = false;
        endTimer.reset();
        thresholdTimer.reset();
        thresholdTimer.start();
        timeoutTimer.reset();
        init = false;
    }

    /**
     * Main runtime, running the intake and handoff subsystems and
     * monitoring for note detection based on current draw.
     */
    @Override
    public void execute() {
        intakeSubsystem.run(0.85);
        handoffSubsystem.run(0.4);

        // Update baseline current draw after 0.5 seconds
        if (thresholdTimer.hasElapsed(0.5) && !init) {
            intakeSubsystem.updateBaselineCurrentDraw();
            init = true;
        }

        // Detect the note and start handoff if detected
        if (intakeSubsystem.noteDetectedByCurrent() && thresholdTimer.hasElapsed(0.5)) {
            notePhaseOne = true;
            timeoutTimer.start();
        }

        // Stop the intake and run the handoff once the note has passed
        if (notePhaseOne && !intakeSubsystem.noteDetectedByCurrent()) {
            intakeSubsystem.run(0);
            handoffSubsystem.run(0.5);
            endTimer.start();
        }

        // End the command after 0.3 seconds of handoff operation
        if (endTimer.hasElapsed(0.3)) {
            done = true;
        }
    }

    /**
     * Ends the command by stopping both subsystems and resetting timers.
     * 
     * @param interrupted whether the command was interrupted before finishing
     */
    @Override
    public void end(boolean interrupted) {
        handoffSubsystem.run(0);
        intakeSubsystem.run(0);
        handoffSubsystem.brake();
        intakeSubsystem.brake();
        thresholdTimer.stop();
        timeoutTimer.stop();
        endTimer.stop();
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