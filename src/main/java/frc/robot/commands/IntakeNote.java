// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.handoff.HandoffSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class IntakeNote extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final HandoffSubsystem handoff;
    private final XboxController driver;
    private final XboxController operator;
    private final Timer thresholdTimer;
    private final Timer timeoutTimer;
    private final Timer endTimer;
    // private final double VOLTAGE_THRESHOLD = 0.5; // need to adjust based on the voltage readings [OUTDATED - LOOK IN INTAKE SUBSYSTEM]
    private final double MAX_INTAKE_TIME = 2.0; 
    private boolean init;
    private boolean notePhaseOne;
    private boolean done;

    public IntakeNote(IntakeSubsystem intakeSubsystem, HandoffSubsystem handoff, XboxController driver, XboxController operator) {
        this.intakeSubsystem = intakeSubsystem;
        this.handoff  = handoff;
        this.driver = driver;
        this.operator = operator;
        this.thresholdTimer = new Timer();
        this.timeoutTimer = new Timer();
        this.endTimer = new Timer();
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.run(0.85);
        handoff.run(0.4);
        if(thresholdTimer.hasElapsed(0.5) && (init == false)){
            intakeSubsystem.updateBaselineCurrentDraw();
            init = true;
        }
        if(intakeSubsystem.noteDetectedByCurrent() && thresholdTimer.hasElapsed(0.5)){
            notePhaseOne = true;
            timeoutTimer.start();
        }
        if(notePhaseOne && !intakeSubsystem.noteDetectedByCurrent()) {
            intakeSubsystem.setSpeed(0);
            intakeSubsystem.setSerializerSpeed(0.5);
            endTimer.start();
        }
        if(endTimer.hasElapsed(0.3)){
            done = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSerializerSpeed(0);
        intakeSubsystem.setSpeed(0);
        intakeSubsystem.brakeSerializer();
        intakeSubsystem.brake();
        thresholdTimer.stop();
        timeoutTimer.stop();
        endTimer.stop();
        if(DriverStation.isTeleop()){
            new ParallelCommandGroup(new PulseRumble(driver), new PulseRumble(operator)).schedule();
        }   
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // with beam break
        // return intakeSubsystem.frontBeamBreakIsTriggered();
        return done || timeoutTimer.hasElapsed(MAX_INTAKE_TIME);
    }
}