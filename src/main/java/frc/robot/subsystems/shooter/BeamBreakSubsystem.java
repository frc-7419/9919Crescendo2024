package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The BeamBreakSubsystem monitors the state of beam break sensors used in the shooter subsystem.
 * These sensors are used to detect the presence of game pieces at specific points within the robot.
 */
public class BeamBreakSubsystem extends SubsystemBase {
    private final DigitalInput beamBreakFront;
    private final DigitalInput beamBreakBack;

    /**
     * Constructor for the BeamBreakSubsystem.
     * Initializes the front and back beam break sensors using the channels defined in Constants.
     */
    public BeamBreakSubsystem() {
        beamBreakFront = new DigitalInput(Constants.BeambreakConstants.frontBeambreakChannel);
        beamBreakBack = new DigitalInput(Constants.BeambreakConstants.backBeambreakChannel);
    }

    /**
     * Checks if the front beam break sensor is triggered.
     * 
     * @return true if the front beam break sensor is triggered, false otherwise.
     */
    public boolean frontBeamBreakIsTriggered() {
        return !beamBreakFront.get();  // The sensor returns false when the beam is broken
    }

    /**
     * Checks if the back beam break sensor is triggered.
     * 
     * @return true if the back beam break sensor is triggered, false otherwise.
     */
    public boolean backBeamBreakIsTriggered() {
        return !beamBreakBack.get();  // The sensor returns false when the beam is broken
    }

    /**
     * This method is called once per scheduler run.
     * It updates the SmartDashboard with the current state of the beam break sensors.
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("frontBeamBreakTriggered", frontBeamBreakIsTriggered());
        SmartDashboard.putBoolean("backBeamBreakTriggered", backBeamBreakIsTriggered());
    }
}
