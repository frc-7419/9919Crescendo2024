package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
  private final DigitalInput beamBreakFront;
  private final DigitalInput beamBreakBack;

  public BeamBreakSubsystem(){
    beamBreakFront = new DigitalInput(0);
    beamBreakBack = new DigitalInput(2);
  }

  public boolean frontBeamBreakIsTriggered(){
    return !beamBreakFront.get();
  }

  public boolean backBeamBreakIsTriggered(){
    return !beamBreakBack.get();
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("frontBeamBreakTriggered",frontBeamBreakIsTriggered());
    SmartDashboard.putBoolean("backBeamBreakTriggered",backBeamBreakIsTriggered());
  }
}
