package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreakSubsystem extends SubsystemBase {
    private final DigitalInput beamBreakFront;
    private final DigitalInput beamBreakBack;

    public BeamBreakSubsystem() {
        beamBreakFront = new DigitalInput(Constants.BeambreakConstants.frontBeambreakChannel);
        beamBreakBack = new DigitalInput(Constants.BeambreakConstants.backBeambreakChannel);
    }
    /**
     * Beam brake composed of two pieces, receiver and laser 
     * Front beam brake is the laser
     */
    public boolean frontBeamBreakIsTriggered() {
        return !beamBreakFront.get();
    }
   /**
    * Beam break composed of two pieces, reiver and laser
    
    * Set as True while idle, while the note is in --> changes to False 
    * ! Flips the false changing to true
    */
    public boolean backBeamBreakIsTriggered() {
        return !beamBreakBack.get();
       /** Set as True while idle, while the note is in --> changes to False 
        * ! Flips the false changing to true
        * Back beam break is the reciever
        */
    }

    @Override
    /** lets coach know whether beambrake is triggered or not */
    public void periodic() {
        SmartDashboard.putBoolean("frontBeamBreakTriggered", frontBeamBreakIsTriggered());
        SmartDashboard.putBoolean("backBeamBreakTriggered", backBeamBreakIsTriggered());
        
    }
}
