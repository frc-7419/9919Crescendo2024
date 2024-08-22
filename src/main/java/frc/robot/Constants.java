// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 */
public final class Constants {

    /**
     * Constants related to the robot's hardware configuration.
     */
    public static class RobotConstants {
        /** The CAN bus name. */
        public static final String kCanbus = "rio"; // TODO: check if we are going to use roborio canbus or a different one like canivore
    }

    /**
     * Constants related to the operator controllers.
     */
    public static class ControllerConstants {
        /** The port for the drive controller. */
        public static final int kDriveControllerPort = 0;
        /** The port for the operator controller. */
        public static final int kOperatorControllerPort = 1;
        /** The default rumble intensity for the controllers. */
        public static final double defaultRumbleIntensity = 0.7;
        /** The default pulse interval for controller rumble feedback. */
        public static final double defaultPulseInterval = 0.5;
    }

    /**
     * Constants related to the elevator subsystem.
     */
    public static class ElevatorConstants {
        /** The ID of the first motor in the elevator subsystem. */
        public static final int motorOneID = 0; // TODO: set ID once robot is built
        /** The ID of the second motor in the elevator subsystem. */
        public static final int motorTwoID = 0; // TODO: set ID once robot is built
    }

    /**
     * Constants related to the amplifier subsystem.
     */
    public static class AmpConstants {
        /** The ID of the top shooter motor. */
        public static final int topShooterID = 0; // TODO: set ID once robot is built
        /** The ID of the bottom shooter motor. */
        public static final int bottomShooterID = 0; // TODO: set ID once robot is built
    }

    /**
     * Constants related to the handoff subsystem.
     */
    public static class HandoffConstants {
        /** The ID of the first motor in the handoff subsystem. */
        public static final int motorOneID = 0; // TODO: set ID once robot is built
        /** The ID of the second motor in the handoff subsystem. */
        public static final int motorTwoID = 0; // TODO: set ID once robot is built
    }

    /**
     * Constants related to the beambreak sensors.
     */
    public static class BeambreakConstants {
        /** The channel for the front beambreak sensor. */
        public static final int frontBeambreakChannel = 0; // TODO: set channel once robot is built
        /** The channel for the back beambreak sensor. */
        public static final int backBeambreakChannel = 2; // TODO: set channel once robot is built
    }

    /**
     * Constants related to the shooter subsystem.
     */
    public static class ShooterConstants {
        /** The ID of the bottom shooter motor. */
        public static final int bottomShooterID = 0; // TODO: set ID once robot is built
        /** The ID of the top shooter motor. */
        public static final int topShooterID = 0; // TODO: set ID once robot is built
    }

    /**
     * Constants related to the intake subsystem.
     */
    public static class IntakeConstants {
        /** The ID of the top intake motor. */
        public static final int topIntakeID = 0; // TODO: set ID once robot is built
        /** The ID of the bottom intake motor. */
        public static final int bottomIntakeID = 0; // TODO: set ID once robot is built
        /** The ID of the intake wrist motor. */
        public static final int intakeWristMotorID = 0; // TODO: set ID once robot is built
    }

    /**
     * Constants related to the swerve drive subsystem.
     */
    public static class SwerveConstants {

        /**
         * Gear ratio for the angle motor in the swerve drive.
         */
        public static final double kGearRatioAngleMotor = 12.0 / 24.0 * 14.0 / 72.0;

        /**
         * Gear ratio for the speed motor in the swerve drive.
         */
        public static final double kSpeedMotorGearRatio = 12.0 / 24.0 * 24.0 / 22.0 * 15.0 / 45.0;

        /** The length of the robot in meters. */
        public static final double LENGTH = Units.inchesToMeters(26.5);

        /** Half of the robot's length in meters. */
        public static final double HALF_LENGTH = LENGTH / 2.0;

        /** Constants for the front left swerve module. */
        public static final SwerveModuleConstants frontLeft = new SwerveModuleConstants(2, 1, 9, 102.39,
                new Translation2d(SwerveConstants.HALF_LENGTH, SwerveConstants.HALF_LENGTH));

        /** Constants for the front right swerve module. */
        public static final SwerveModuleConstants frontRight = new SwerveModuleConstants(4, 3, 10, 189.94,
                new Translation2d(SwerveConstants.HALF_LENGTH, -SwerveConstants.HALF_LENGTH));

        /** Constants for the back left swerve module. */
        public static final SwerveModuleConstants backLeft = new SwerveModuleConstants(8, 7, 12, 161.63,
                new Translation2d(-SwerveConstants.HALF_LENGTH, SwerveConstants.HALF_LENGTH));

        /** Constants for the back right swerve module. */
        public static final SwerveModuleConstants backRight = new SwerveModuleConstants(6, 5, 11, 246.09,
                new Translation2d(-SwerveConstants.HALF_LENGTH, -SwerveConstants.HALF_LENGTH));

        /** The swerve drive kinematics for the robot. */
        public static final SwerveDriveKinematics m_SwerveDriveKinematics = new SwerveDriveKinematics(
                SwerveConstants.frontLeft.location, SwerveConstants.frontRight.location, SwerveConstants.backLeft.location,
                SwerveConstants.backRight.location);

        /** The maximum translational speed of the robot in meters per second. */
        public static final double kMaxTranslationalSpeed = Units.feetToMeters(3);

        /** The maximum rotational speed of the robot in radians per second. */
        public static final double kMaxRotationalSpeed = Math.PI / 4;

        /** The diameter of the wheels in meters. */
        public static final double kWheelDiameter = Units.inchesToMeters(3.5);

        /** The circumference of the wheels in meters. */
        public static final double kWheelCircumfrence = kWheelDiameter * Math.PI;

        /** The proportional gain for the angle PID controller. */
        public static final double anglekP = 0.002;

        /** The integral gain for the angle PID controller. */
        public static final double anglekI = 0;

        /** The derivative gain for the angle PID controller. */
        public static final double anglekD = 0;
    }

    /**
     * Constants related to individual swerve modules.
     */
    public static class SwerveModuleConstants {
        /** The maximum turning speed of the swerve module. */
        public static final double kMaxTurningSpeed = 0.3;

        /** The ID of the drive motor for the swerve module. */
        public final int driveMotorID;

        /** The ID of the turn motor for the swerve module. */
        public final int turnMotorID;

        /** The ID of the turn encoder for the swerve module. */
        public final int turnEncoderID;

        /** The offset for the turn encoder in degrees. */
        public final double offset;

        /** The location of the swerve module relative to the robot center. */
        public final Translation2d location;

        /**
         * Constructs a new SwerveModuleConstants instance.
         *
         * @param driveMotorID    the ID of the drive motor
         * @param turnMotorID     the ID of the turn motor
         * @param turnEncoderID   the ID of the turn encoder
         * @param offset          the offset for the turn encoder
         * @param location        the location of the module relative to the robot center
         */
        public SwerveModuleConstants(int driveMotorID, int turnMotorID, int turnEncoderID, double offset, Translation2d location) {
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.turnEncoderID = turnEncoderID;
            this.offset = offset;
            this.location = location;
        }
    }
}
