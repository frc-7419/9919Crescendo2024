package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightRangeChecker extends SubsystemBase {
    private final int blueSpeakerFiducialID = 7;
    private final int redSpeakerFiducialID = 4;
    private final double limelightMountAngleDegrees = 25.0; 
    private double limelightLensHeightInches = 20.0; 
    private double goalHeightInches = 60.0; 

    public LimelightRangeChecker() {}

    public double getDistance(LimelightHelpers.LimelightTarget_Fiducial fiducial) {
        double targetOffsetAngle_Vertical = fiducial.ty;
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }
    

    public boolean speakerFiducialInRange(int targetRange) {
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight"); //getting the json data from limelight
        Pose2d botPose = LimelightHelpers.getBotPose2d("limelight");
        if (llresults == null || llresults.targets_Fiducials == null) { //making sure its not null to prevent code from breaking robot
            SmartDashboard.putString("Currently", "No Limelight results found");
            System.out.println("NOTHING FOUND :-(");
            return false;
        } else {
        
            System.out.println(llresults.targets_Fiducials[0].fiducialID);
            // Iterate through fiducials to find speaker fiducial
            System.out.println(llresults.targets_Fiducials.length);
            for (LimelightHelpers.LimelightTarget_Fiducial fiducial : llresults.targets_Fiducials) {
                double distance = getDistance(fiducial);//calls getdistance function with fiducial as argument
                if ((DriverStation.getAlliance().toString().equalsIgnoreCase("blue") && fiducial.fiducialID == blueSpeakerFiducialID) ||
                    (DriverStation.getAlliance().toString().equalsIgnoreCase("red") && fiducial.fiducialID == redSpeakerFiducialID)) {
    
                    SmartDashboard.putString("Currently", "Speaker fiducial detected at distance: " + distance);
                    if (distance <= targetRange) {
                        SmartDashboard.putString("Currently", "Speaker fiducial is in range");
                        return true;
                    }
                } else {
                    
                    SmartDashboard.putString("Currently", "No speaker fiducial detected, other fiducials at distance: " + distance);
                }
            }
        }
        return false;
    }
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
