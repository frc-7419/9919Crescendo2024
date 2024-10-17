package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;  // Import for the Pigeon2 gyro
import edu.wpi.first.math.geometry.Pose2d;  // Import for representing robot's position
import edu.wpi.first.math.geometry.Rotation2d;  // Import for representing rotation
import edu.wpi.first.math.kinematics.ChassisSpeeds;  // Import for handling chassis speeds
import edu.wpi.first.math.kinematics.SwerveModulePosition;  // Import for swerve module positions
import edu.wpi.first.math.kinematics.SwerveModuleState;  // Import for swerve module states
import edu.wpi.first.wpilibj2.command.SubsystemBase;  // Base class for a subsystem
import frc.robot.Constants;  // Import constants for the robot
import frc.robot.Constants.SwerveConstants;  // Import swerve-specific constants
import frc.robot.LimelightHelpers;  // Import helpers for Limelight integration
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  // For SmartDashboard updates
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;  // Import for pose estimation
import edu.wpi.first.math.VecBuilder;  // Import for vector building

// BackupDriveBaseSubsystem class that provides basic swerve drive functionalities
public class BackupDriveBaseSubsystem extends SubsystemBase {
    private final SwerveModule frontLeftModule;  // Front left module
    private final SwerveModule frontRightModule; // Front right module
    private final SwerveModule backLeftModule;   // Back left module
    private final SwerveModule backRightModule;  // Back right module
    private final Pigeon2 gyro;  // Declare the gyro variable
    private final SwerveDrivePoseEstimator m_poseEstimator;  // Pose estimator for the robot

    // Constructor for BackupDriveBaseSubsystem
    public BackupDriveBaseSubsystem() {
        // Initialize the gyro
        gyro = new Pigeon2(0);  // Use the appropriate port for the Pigeon2 gyro
        gyro.reset();  // Reset the gyro to start from a known orientation

        // Initialize front left swerve module with configuration
        frontLeftModule = new SwerveModule(SwerveConstants.frontLeft.turnMotorID,
                SwerveConstants.frontLeft.driveMotorID, SwerveConstants.frontLeft.turnEncoderID,
                SwerveConstants.frontLeft.offset, "FrontLeftModule");
        
        // Initialize front right swerve module
        frontRightModule = new SwerveModule(SwerveConstants.frontRight.turnMotorID,
                SwerveConstants.frontRight.driveMotorID, SwerveConstants.frontRight.turnEncoderID,
                SwerveConstants.frontRight.offset, "FrontRightModule");
        
        // Initialize back left swerve module
        backLeftModule = new SwerveModule(SwerveConstants.backLeft.turnMotorID,
                SwerveConstants.backLeft.driveMotorID, SwerveConstants.backLeft.turnEncoderID,
                SwerveConstants.backLeft.offset, "BackLeftModule");
        
        // Initialize back right swerve module
        backRightModule = new SwerveModule(SwerveConstants.backRight.turnMotorID,
                SwerveConstants.backRight.driveMotorID, SwerveConstants.backRight.turnEncoderID,
                SwerveConstants.backRight.offset, "BackRightModule");

        // Initialize the pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
                Constants.SwerveConstants.m_SwerveDriveKinematics,
                gyro.getRotation2d(),  // Use gyro for initial rotation
                getPositions(),  // Get initial positions of the swerve modules
                new Pose2d(),  // Start from an initial pose of (0, 0, 0)
                VecBuilder.fill(0.1, 0.1, 0.1),  // Placeholder for measurement noise
                VecBuilder.fill(0.1, 0.1, 0.1)   // Placeholder for vision measurement noise
        );
    }
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPose(), 
            frontRightModule.getPose(),
            backLeftModule.getPose(),
            backRightModule.getPose()
        };
    public void zeroYaw() {
        gyro.reset();
    }

    public double getYaw() { // CW IS POSITIVE BY DEFAULT
        return -gyro.getAngle();
    }

    public double getPitch() {
        return gyro.getPitch().getValue();
    }

    public double getRoll() {
        return gyro.getRoll().getValue();
    }

    // Method to drive the robot using joystick input
    public void drive(double vx, double vy, double rotation) {
        // Create an array of swerve module states based on joystick inputs
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(vx, Rotation2d.fromDegrees(rotation)),  // Front left module state
            new SwerveModuleState(vx, Rotation2d.fromDegrees(rotation)),  // Front right module state
            new SwerveModuleState(vx, Rotation2d.fromDegrees(rotation)),  // Back left module state
            new SwerveModuleState(vx, Rotation2d.fromDegrees(rotation))   // Back right module state
        };
        // Set the states for each swerve module
        setModuleStates(states);
    }

    // Private method to set the state of each swerve module
    private void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeftModule.setSwerveModuleState(moduleStates[0]);  // Set front left module state
        frontRightModule.setSwerveModuleState(moduleStates[1]); // Set front right module state
        backLeftModule.setSwerveModuleState(moduleStates[2]);   // Set back left module state
        backRightModule.setSwerveModuleState(moduleStates[3]);  // Set back right module state
    }

    public void brake() {
        frontLeftModule.brake();
        frontRightModule.brake();
        backLeftModule.brake();
        backRightModule.brake();
    }

    public void coast() {
        frontLeftModule.coast();
        frontRightModule.coast();
        backLeftModule.coast();
        backRightModule.coast();
    }
    public void stop() {
        frontLeftModule.stop();  
        frontRightModule.stop(); 
        backLeftModule.stop();   
        backRightModule.stop();  
    }

    // Method to estimate pose from Limelight
    public void estimatePoseFromLimelight() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (limelightMeasurement.tagCount >= 2) {
            Pose2d pose = limelightMeasurement.pose;
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            m_poseEstimator.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
        }
    }

    // Periodic method called regularly to update the subsystem
    @Override
    public void periodic() {
        // Output status to SmartDashboard for monitoring
        SmartDashboard.putString("Backup Drive Status", "Active");
    }
}
