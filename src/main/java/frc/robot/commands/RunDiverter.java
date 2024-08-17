// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handoff.HandoffSubsystem;

public class RunDiverter extends Command {
    // Just apply a given power to the diverter wheel motor.

    // Example call: operator.y().whileTrue(new RunDiverter(diverter, 0.5));

    // Declare the subsystem and variables

    private final HandoffSubsystem handoffSubsystem;
    private final float voltage;

    public RunDiverter(HandoffSubsystem handoffSubsystem, float voltage) {
        // Initialize the subsystem and variables
        this.handoffSubsystem = handoffSubsystem;
        this.voltage = voltage;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(handoffSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Initialize the motor
        handoffSubsystem.coast();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Run the motor
        handoffSubsystem.setVoltage(voltage);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the motor
        handoffSubsystem.brake();
        handoffSubsystem.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Stop when condition is met such as a beam break sensor. Not sure what will be
        // used but use a beam break sensor as a placeholder.
        return false;
    }
}
