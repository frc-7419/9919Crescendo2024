// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    // Limelight 3 on shooter
    // Limelight 2 with Google Coral on Intake side
    public static class RobotConstants {
        public static final String kCanbus = "rio";
        public static final double kRobotLength = Units.inchesToMeters(34.75); //TODO +- .25
        public static final double kRobotWidth = Units.inchesToMeters(26.5); //TODO
        public static final double kRobotHalfLength = kRobotLength / 2.0; 
        public static final double kRobotWeight = 80; //TODO +- 40
        public static final double kRobotHeight = Units.inchesToMeters(22); //from the floor
        public static final double kFramePerimeter = Units.inchesToMeters(122.5);
    }

    public static class ControllerConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double defaultRumbleIntensity = 0.7;
        public static final double defaultPulseInterval = 0.5;
    }

    public static class HandoffConstants {
        public static final int loaderID = 0; //TODO: set ID once robot is built
        // 1:1 gear ratio
    }

    public static class BeambreakConstants {
        public static final int frontBeambreakChannel = 0; //TODO: set channel once robot is built
        public static final int backBeambreakChannel = 0; //TODO: set channel once robot is built
    }

    public static class ShooterConstants {
        public static final int bottomShooterID = 0; //TODO: set ID once robot is built
        public static final int topShooterID = 0; //TODO: set ID once robot is built
        public static final double shooterGearRatio = 24/18; // Motor to wheel
    }

    public static class IntakeConstants {
        public static final int intakeID = 0; //TODO: set ID once robot is built
        public static final double intakeGearRatio = 30/24; // Motor to wheel

    }

    public static class SwerveConstants {

        public static final double kWidthBetweenModules = 19.25;
        public static final double kLengthBetweenModules = 21.25;


        /*
         * IMPORTANT: THIS WAS FOUND THROUGH CAD FILES BUT THERE ARE MANY SWERVE X
         * CONFIGURATIONS
         * SO YOU NEED TO DOUBLE CHECK THIS IS CORRECT IN PRACTICE
         */
        /*
         * ANGLE MOTOR
         * NEO Shaft to 12T Pulley to 24T Pulley to 14T Gear to 72T Main Rotation Gear
         */
        public static final double kGearRatioTurnMotor = 72/7;
        // 12.0 / 24.0 * 14.0 / 72.0; //OLD
        /*
         * DRIVE MOTOR
         * NEO shaft to 12T Pulley to 24T Pulley to 24T Gear to 22T Gear to 15T bevel to
         * 45T Bevel
         *
         * The CANCODER measures rotations of a the driven 1:1 PULLEY in which the
         * driver pulley is on the same
         * shaft as the 24T Pulley
         */
        public static final double kDriveMotorGearRatio = 6;
        // 12.0 / 24.0 * 24.0 / 22.0 * 15.0 / 45.0; //OLD
        public static final double kWheelDiameter = Units.inchesToMeters(4); 
        public static final double kWheelCircumfrence = kWheelDiameter * Math.PI; //TODO
        public static final double kMaxTranslationalSpeed = Units.feetToMeters(3); //TODO
        public static final double kMaxRotationalSpeed = Math.PI / 4; // arbitrary value in radians, let's say one pi/second  //TODO
        public static final double anglekP = 0.002;
        public static final double anglekI = 0;
        public static final double anglekD = 0;
        /* INFO: according to WPILib docs "The locations for the modules must be
         * relative to the center of the robot. Positive x
         * values represent moving toward the front of the robot whereas positive y
         * values represent moving toward the left of the robot."
         */
        public static final SwerveModuleConstants frontLeft = new SwerveModuleConstants(2, 1, 9, 102.39,
                new Translation2d(RobotConstants.kRobotHalfLength, RobotConstants.kRobotHalfLength));
        public static final SwerveModuleConstants frontRight = new SwerveModuleConstants(4, 3, 10, 189.94,
                new Translation2d(RobotConstants.kRobotHalfLength, -RobotConstants.kRobotHalfLength));
        public static final SwerveModuleConstants backLeft = new SwerveModuleConstants(8, 7, 12, 161.63,
                new Translation2d(-RobotConstants.kRobotHalfLength, RobotConstants.kRobotHalfLength));
        public static final SwerveModuleConstants backRight = new SwerveModuleConstants(6, 5, 11, 246.09,
                new Translation2d(-RobotConstants.kRobotHalfLength, -RobotConstants.kRobotHalfLength));
        public static final SwerveDriveKinematics m_SwerveDriveKinematics = new SwerveDriveKinematics(
                SwerveConstants.frontLeft.location, SwerveConstants.frontRight.location,
                SwerveConstants.backLeft.location,  SwerveConstants.backRight.location);

    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int turnMotorID;
        public final int turnEncoderID;
        public final double offset;
        public final Translation2d location;

        public SwerveModuleConstants(final int driveMotorID, final int turnMotorID, final int turnEncoderID, final double offset, final Translation2d location) {
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.turnEncoderID = turnEncoderID;
            this.offset = offset;
            this.location = location;
        }
    }
    
}