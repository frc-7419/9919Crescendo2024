// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TEST FILE

package frc.robot.subsystems.haptics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.haptics.HapticRequest.RequestType;

public class ControlHapticsWithJoystick extends Command {
    private final Haptics haptics;
    private final CommandXboxController joystick;

    /**
     * Creates a new ControlHapticsWithJoystick.
     * @param haptics the haptics subsystem
     * @param joystick the joystick to control the haptics
     */
    public ControlHapticsWithJoystick(Haptics haptics, CommandXboxController joystick) {
        this.haptics = haptics;
        this.joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(haptics);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (joystick.a().getAsBoolean()) {
            haptics.startRumble(Haptics.ControllerType.DRIVER, 1);
            System.out.println("Rumble");
        }
        if (joystick.b().getAsBoolean()) {
            haptics.setRequest(new HapticRequest(Haptics.ControllerType.DRIVER, RequestType.STOP));
        }
        if (joystick.x().getAsBoolean()) {
            haptics.setRequest(new HapticRequest(Haptics.ControllerType.BOTH, RequestType.WARN, 3));
        }
        if (joystick.y().getAsBoolean()) {
            haptics.setRequest(new HapticRequest(Haptics.ControllerType.DRIVER, RequestType.RUMBLE, 3));
        }
    }

    // Called once the command ends or is interrupted.
    /*
     * Ends the command.
     * @param interrupted whether the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        haptics.setRequest(new HapticRequest(Haptics.ControllerType.DRIVER, RequestType.STOP));
    }

    // Returns true when the command should end.
    /*
     * Checks if the command is finished.
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
