// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
    // Limelight 3 on shooter
    // Limelight 2 with Google Coral on Intake side
    public static class RobotConstants {
        public static final String kCanbus = "rio";
        public static final double kRobotLength = Units.inchesToMeters(34.75); // TODO +- .25
        public static final double kRobotWidth = Units.inchesToMeters(26.5); // TODO
        public static final double kRobotHalfLength = kRobotLength / 2.0;
        public static final double kRobotWeight = 80; // TODO +- 40
        public static final double kRobotHeight = Units.inchesToMeters(22); // from the floor
        public static final double kFramePerimeter = Units.inchesToMeters(122.5);
    }

    public static class ControllerConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double defaultRumbleIntensity = 0.7;
        public static final double defaultPulseInterval = 0.5;
    }

    public static class BeambreakConstants {
        public static final int frontBeambreakChannel = 0; // TODO: set channel once robot is built
        public static final int backBeambreakChannel = 0; // TODO: set channel once robot is built
    }

    public static class ShooterConstants {
        public static final int bottomShooterID = 10; // TODO: set ID once robot is built
        public static final int topShooterID = 11; // TODO: set ID once robot is built
        public static final double shooterGearRatio = 24 / 18; // Motor to wheel
        public static final double topShooterRPM = 10000;
        public static final double bottomShooterRPM = 10000;
    }

    public static class IntakeConstants {
        public static final int intakeID = 11;
        public static final double intakeGearRatio = 30 / 24; // Motor to wheel
        public static final double CURRENT_THRESHOLD = 15.0; // TODO: set threshold
        public static final double MAX_INTAKE_TIME = 5.0; // TODO: set time
        public static final double INTAKE_POWER = 0.35; // Constant power for intake
    }

    public static class SwerveConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    }
}