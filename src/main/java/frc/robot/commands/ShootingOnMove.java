package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootingOnMove extends Command {
  private final DriveBaseSubsystem driveBaseSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PIDController aimController;
  private static final double aimTolerance = 1.0; // degrees
  private static final double maxSpeedAdjustment = 0.2;
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.1;
  private static final double NOMINAL_SHOOTER_SPEED = 0.9;
  private static final double COMPENSATION_FACTOR = 0.1; // Proportional factor for velocity compensation
  private final XboxController joystick;

  public ShootingOnMove(DriveBaseSubsystem driveBaseSubsystem, ShooterSubsystem shooterSubsystem,
      XboxController joystick) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.aimController = new PIDController(kP, kI, kD);
    this.aimController.setTolerance(aimTolerance);
    this.joystick = joystick;
    addRequirements(driveBaseSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    aimController.reset();
  }

  @Override
  public void execute() {
    ChassisSpeeds currentSpeeds = driveBaseSubsystem.getChassisSpeedsFromJoystick(joystick.getLeftY(),
        joystick.getLeftX(), 0, joystick.getLeftBumper());

    double robotVelocityX = currentSpeeds.vxMetersPerSecond; // Forward velocity of the robot
    double tx = LimelightHelpers.getTX("limelight");
    double rotationAdjustment = aimController.calculate(tx, 0);

    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
        currentSpeeds.vxMetersPerSecond,
        currentSpeeds.vyMetersPerSecond,
        currentSpeeds.omegaRadiansPerSecond + rotationAdjustment);

    driveBaseSubsystem.setModuleStates(adjustedSpeeds);

    // Manually calculate the distance to the target using Limelight
    double distanceToTarget = getDistanceToTarget();

    // Manually calculate the shooter speed based on the distance
    double baseShooterSpeed = getShooterSpeedForDistance(distanceToTarget);

    // Adjust shooter speed to compensate for robot's forward velocity
    double adjustedShooterSpeed = baseShooterSpeed + (robotVelocityX * COMPENSATION_FACTOR);

    // Ensure the adjusted shooter speed stays within bounds (e.g., max shooter
    // speed = 1.0)
    adjustedShooterSpeed = Math.min(1.0, adjustedShooterSpeed);

    // Set the shooter speed
    shooterSubsystem.run(adjustedShooterSpeed, adjustedShooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Manual function to calculate distance to target using Limelight data
  public double getDistanceToTarget() {
    double targetHeight = 2.5; // Example: height of the target in meters (e.g., high goal)
    double cameraHeight = 1.0; // Example: height of your Limelight in meters
    double cameraAngle = 25.0; // Example: angle of your Limelight from horizontal in degrees
    double ty = LimelightHelpers.getTY("limelight"); // Vertical offset angle from Limelight

    double angleToTargetRadians = Math.toRadians(cameraAngle + ty);

    return (targetHeight - cameraHeight) / Math.tan(angleToTargetRadians);
  }

  // Manual function to calculate shooter speed based on distance to target
  public double getShooterSpeedForDistance(double distance) {
    double minSpeed = 0.6; // Example: minimum shooter speed at the closest range
    double maxSpeed = 1.0; // Example: maximum shooter speed at farthest range
    double minDistance = 2.0; // Example: minimum shooting distance in meters
    double maxDistance = 8.0; // Example: maximum shooting distance in meters

    double k = (maxSpeed - minSpeed) / (maxDistance - minDistance);

    double shooterSpeed = minSpeed + k * (distance - minDistance);

    return Math.min(maxSpeed, Math.max(minSpeed, shooterSpeed));
  }
}
