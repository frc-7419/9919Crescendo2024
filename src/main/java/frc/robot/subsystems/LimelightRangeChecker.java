package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightRangeChecker extends SubsystemBase {
    private final int blueSpeakerFiducialID = 7;
    private final int redSpeakerFiducialID = 4;

    public LimelightRangeChecker() {}


    public boolean speakerFiducialInRange(int targetRange) {
        PoseEstimate poseEstimate; //making a PoseEstimate, which is what limeleight helpers uses for pose stuff
        if (DriverStation.getAlliance().toString().equalsIgnoreCase("blue")) { // getting pose estimate based on alliance color
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
        }
        if (poseEstimate.rawFiducials.length == 0) { // making sure there are fiducials detected
            SmartDashboard.putString("Currently", " no fiducials detected"); // for debugging purposes
            return false;
        }
        //limelight helpers has this weird system where theres the pose estimate, and then in them theres many fiducials(as an array of rawfiducials), and then for each one you can get the id and distance(and a bunch of otehr very helpful stuff)
        for (RawFiducial fiducial : poseEstimate.rawFiducials) {
            if ((DriverStation.getAlliance().toString().equalsIgnoreCase("blue") && fiducial.id == blueSpeakerFiducialID) ||
                (DriverStation.getAlliance().toString().equalsIgnoreCase("red")&& fiducial.id == redSpeakerFiducialID)) {
                SmartDashboard.putString("Currently", " speaker fiducial detected at distance(not in range): " + fiducial.distToCamera); // for debugging
                if (fiducial.distToCamera <= targetRange) { //make sure its in range
                    SmartDashboard.putString("Currently", " speaker fiducial is in range"); // for debugging
                    return true;
                }
                
            }else{
                SmartDashboard.putString("Currently", " no fiducial with speaker id detected, other fiducials detected");
            }
        }
        return false; 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
