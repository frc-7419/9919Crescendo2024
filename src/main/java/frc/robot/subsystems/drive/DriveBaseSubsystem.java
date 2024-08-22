// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The DriveBaseSubsystem class is responsible for controlling the robot's drive base.
 * It includes functionality for handling swerve modules, odometry, and the gyro (AHRS).
 */
public class DriveBaseSubsystem extends SubsystemBase {
    private final SwerveDriveOdometry m_odometry;
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private final AHRS ahrs;

    /**
     * Constructs a new DriveBaseSubsystem.
     * Initializes swerve modules, the AHRS gyro, and sets up odometry.
     */
    public DriveBaseSubsystem() {
        frontLeftModule = new SwerveModule(SwerveConstants.frontLeft.turnMotorID, SwerveConstants.frontLeft.driveMotorID, SwerveConstants.frontLeft.turnEncoderID, SwerveConstants.frontLeft.offset, "FrontLeftModule");
        frontRightModule = new SwerveModule(SwerveConstants.frontRight.turnMotorID, SwerveConstants.frontRight.driveMotorID, SwerveConstants.frontRight.turnEncoderID, SwerveConstants.frontRight.offset, "FrontRightModule");
        backLeftModule = new SwerveModule(SwerveConstants.backLeft.turnMotorID, SwerveConstants.backLeft.driveMotorID, SwerveConstants.backLeft.turnEncoderID, SwerveConstants.backLeft.offset, "BackLeftModule");
        backRightModule = new SwerveModule(SwerveConstants.backRight.turnMotorID, SwerveConstants.backRight.driveMotorID, SwerveConstants.backRight.turnEncoderID, SwerveConstants.backRight.offset, "BackRightModule");
        
        ahrs = new AHRS(SerialPort.Port.kMXP);
        ahrs.zeroYaw(); // Field-centric, we need yaw to be zero
        
        m_odometry = new SwerveDriveOdometry(Constants.SwerveConstants.m_SwerveDriveKinematics, ahrs.getRotation2d(), getPositions());
        
        coast();
    }

    /**
     * Resets the yaw of the AHRS gyro to zero.
     */
    public void zeroYaw() {
        ahrs.zeroYaw();
    }

    /**
     * Gets the current yaw angle of the robot.
     * 
     * @return The current yaw angle in degrees, with CW as positive.
     */
    public double getYaw() { // CW IS POSITIVE BY DEFAULT
        return -ahrs.getYaw();
    }

    /**
     * Gets the current pitch angle of the robot.
     * 
     * @return The current pitch angle in degrees.
     */
    public double getPitch() {
        return ahrs.getPitch();
    }

    /**
     * Gets the current roll angle of the robot.
     * 
     * @return The current roll angle in degrees.
     */
    public double getRoll() {
        return ahrs.getRoll();
    }

    /**
     * Checks if the robot has reached a specified distance.
     * 
     * @param meters The distance to check in meters.
     * @return True if all swerve modules have reached the specified distance.
     */
    public boolean reachedDist(double meters) {
        return
            (frontLeftModule.reachedDist(meters)) &&
            (frontRightModule.reachedDist(meters)) &&
            (backLeftModule.reachedDist(meters)) &&
            (backRightModule.reachedDist(meters));
    }

    /**
     * Resets the drive encoders of all swerve modules.
     */
    public void resetDriveEnc() {
        frontLeftModule.resetDriveEncoder();
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();
    }

    /**
     * Gets the current rotation of the robot as a Rotation2d object.
     * 
     * @return The current rotation as a Rotation2d object.
     */
    public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
        /*
         * The thing is .getYaw is -180 to 180 so it not being 0 to 360
         * may cause the internal conversion that Rotation2d does to be wrong
         */
    }

    /**
     * Sets the swerve modules to brake mode.
     */
    public void brake() {
        frontLeftModule.brake();
        frontRightModule.brake();
        backLeftModule.brake();
        backRightModule.brake();
    }

    /**
     * Sets the swerve modules to coast mode.
     */
    public void coast() {
        frontLeftModule.coast();
        frontRightModule.coast();
        backLeftModule.coast();
        backRightModule.coast();
    }

    /**
     * Gets the positions of all swerve modules.
     * 
     * @return An array of SwerveModulePosition objects representing the positions of the modules.
     */
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
            frontLeftModule.getPose(), 
            frontRightModule.getPose(), 
            backLeftModule.getPose(), 
            backRightModule.getPose()
        };
    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose as a Pose2d object.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(ahrs.getRotation2d(), getPositions(), pose);
    }

    /**
     * Returns chassis speeds from field-centric joystick controls. 
     * This determines the translational speed of the robot in proportion to joystick values.
     *
     * @param vx The velocity in the x direction (forward).
     * @param vy The velocity in the y direction (sideways).
     * @param rx The rotational velocity around the z-axis.
     * @param slowMode If true, the robot will move slower.
     * @return The corresponding ChassisSpeeds.
     */
    public ChassisSpeeds getChassisSpeedsFromJoystick(double vx, double vy, double rx, boolean slowMode) {
        vx = Math.abs(vx) > 0.05 ? -vx * SwerveConstants.kMaxTranslationalSpeed : 0;
        vy = Math.abs(vy) > 0.05 ? vy * SwerveConstants.kMaxTranslationalSpeed : 0;
        rx = Math.abs(rx) > 0.05 ? -0.7 * rx * SwerveConstants.kMaxRotationalSpeed : 0;
        if (slowMode) {
            vx *= 0.2;
            vy *= 0.2;
            rx *= 0.2;
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, getRotation2d());
    }

    /**
     * Sets the individual swerve module states.
     *
     * @param moduleStates An array of SwerveModuleState objects representing the desired states for each module.
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeftModule.setSwerveModuleState(moduleStates[0]);
        frontRightModule.setSwerveModuleState(moduleStates[1]);
        backLeftModule.setSwerveModuleState(moduleStates[2]);
        backRightModule.setSwerveModuleState(moduleStates[3]);
    }

    /**
     * Sets the individual swerve module states based on chassis speeds.
     *
     * @param chassisSpeeds The desired ChassisSpeeds.
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(Constants.SwerveConstants.m_SwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * This method is called periodically by the scheduler.
     * It updates the odometry and outputs data to the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Yaw", getYaw());
        frontLeftModule.outputDashboard();
        frontRightModule.outputDashboard();
        backLeftModule.outputDashboard();
        backRightModule.outputDashboard();
        m_odometry.update(getRotation2d(), getPositions());
    }
}
