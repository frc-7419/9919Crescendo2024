// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * This class handles the low-level logic for a swerve module, including motor and encoder management.
 */
public class SwerveModule {
    private final CANSparkMax turnMotor;
    private final CANSparkMax driveMotor;
    // private final CANCoder turnEncoder;
    private final RelativeEncoder driveEncoder;
    private final PIDController angleController;
    private final String module;

    /**
     * Constructs a new SwerveModule which manages one turn motor and one drive motor.
     *
     * @param turnMotorID       CAN ID of the turn motor.
     * @param driveMotorID      CAN ID of the drive motor.
     * @param turnEncoderID     CAN ID of the turn encoder.
     * @param turnEncoderOffset Absolute position offset in degrees.
     * @param module            Name of the module for shuffleboard output.
     */
    public SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderID, double turnEncoderOffset, String module) {
        this.module = module;

        // PID controller initialization
        angleController = new PIDController(Constants.SwerveConstants.anglekP, Constants.SwerveConstants.anglekI, Constants.SwerveConstants.anglekD);
        angleController.enableContinuousInput(0, 360);
        angleController.setTolerance(0.5);

        // Turn motor initialization
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kCoast);

        // Drive motor initialization
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kCoast);

        // Turn encoder initialization
        // turnEncoder = new CANCoder(turnEncoderID);
        // turnEncoder.configFactoryDefault();
        // turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // turnEncoder.configMagnetOffset(turnEncoderOffset);
        // turnEncoder.configSensorDirection(false);

        // Drive encoder initialization
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(1 / ((1 / Constants.SwerveConstants.kWheelCircumfrence) * 5.5));
        driveEncoder.setVelocityConversionFactor(1); // TODO: fix this
    }

    /**
     * Sets the motors to coast mode.
     */
    public void coast() {
        turnMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the motors to brake mode.
     */
    public void brake() {
        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Gets the current position of the drive encoder.
     * 
     * @return The position of the drive encoder in meters.
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Resets the drive encoder's position to zero.
     */
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    /**
     * Checks if the module has reached the specified distance.
     * 
     * @param meters The distance to check in meters.
     * @return True if the module has reached the distance, false otherwise.
     */
    public boolean reachedDist(double meters) {
        return Math.abs(driveEncoder.getPosition()) > meters;
    }

    /**
     * Gets the current pose of the swerve module.
     * 
     * @return The current SwerveModulePosition, including drive position and module angle.
     */
    public SwerveModulePosition getPose() {
        return null;
        // return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current SwerveModuleState, including velocity and module angle.
     */
    public SwerveModuleState getSwerveModuleState() {
        return null;
        // return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }

    /**
     * Sets the state of the swerve module, adjusting the motor outputs to match the desired state.
     * 
     * @param state The desired state of the swerve module.
     */
    public void setSwerveModuleState(SwerveModuleState state) {
        // state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
        // state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition())).getCos();
        driveMotor.set(state.speedMetersPerSecond);
        // turnMotor.set(-MathUtil.clamp(angleController.calculate(turnEncoder.getAbsolutePosition(), state.angle.getDegrees()) , -Constants.SwerveModuleConstants.kMaxTurningSpeed, Constants.SwerveModuleConstants.kMaxTurningSpeed));
    }

    /**
     * Stops all motors in the module.
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Outputs relevant values to the SmartDashboard for debugging and monitoring.
     */
    public void outputDashboard() {
        // SmartDashboard.putNumber(module + " angle", turnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(module + " driveEncoder", getDrivePosition());
    }
}
