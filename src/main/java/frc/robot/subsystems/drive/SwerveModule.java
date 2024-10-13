// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * This is where the low-level logic for swerve is handled
 */
public class SwerveModule {
    private final TalonFX turnMotor;
    private final TalonFX driveMotor;
    private final CANcoder turnEncoder;
    private final String module;
    private boolean isBrakeMode = false;

    /**
     * Makes a new swerve module, this handles one turn motor and one drive motor
     *
     * @param turnMotorID       is a CAN ID parameter (int)
     * @param driveMotorID      is a CAN ID parameter (int)
     * @param turnEncoderID     is a CAN ID parameter (int)
     * @param turnEncoderOffset is absolute pos at zero in deg (double)
     * @param module            for naming modules during comprehensive shuffleboard outputs (String)
     */
    public SwerveModule(final int turnMotorID, final int driveMotorID, final int turnEncoderID, final double turnEncoderOffset, final String module) {
        this.module = module;
        // Turn motor init
        turnMotor = new TalonFX(turnMotorID);
        turnMotor.setNeutralMode(NeutralModeValue.Coast);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.Slot0.kS = 0.1; // To account for friction                                                     TODO: calculate friction
        turnConfig.Slot0.kV = 0.11299435; // volts per rotation per second                                        DONT TOUCH
        turnConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 DONT TOUCH
        turnConfig.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
        turnConfig.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
        turnConfig.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
        turnConfig.Voltage.PeakForwardVoltage = 12; // Maximum forward voltage
        turnConfig.Voltage.PeakReverseVoltage = -12; // Maximum reverse voltage
        turnMotor.getConfigurator().apply(turnConfig);
        // Drive motor init
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0.kS = 0.1; // To account for friction                                                     TODO: calculate friction
        driveConfig.Slot0.kV = 0.11299435; // volts per rotation per second                                        DONT TOUCH
        driveConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 DONT TOUCH
        driveConfig.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
        driveConfig.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
        driveConfig.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
        driveConfig.Voltage.PeakForwardVoltage = 12; // Maximum forward voltage
        driveConfig.Voltage.PeakReverseVoltage = -12; // Maximum reverse voltage
        driveMotor.getConfigurator().apply(driveConfig);
        // Turn encoder init
        turnEncoder = new CANcoder(turnEncoderID);
        turnEncoder.setPosition(turnEncoderOffset);
        CANcoderConfiguration turnEncoderConfiguration = new CANcoderConfiguration();
        // TODO write configuration
        turnEncoder.getConfigurator().apply(turnEncoderConfiguration);
    }
    
    public void toggleMode(){
        NeutralModeValue newMode = isBrakeMode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        isBrakeMode = !isBrakeMode;
        turnMotor.setNeutralMode(newMode);
        driveMotor.setNeutralMode(newMode);
    }

    public void coast() {
        turnMotor.setNeutralMode(NeutralModeValue.Coast);
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        isBrakeMode = false;
    }

    public void brake() {
        turnMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        isBrakeMode = true;
    }

    public void resetDriveEncoder() {
        driveMotor.setPosition(0);
    }

    public boolean reachedDist(double meters) {
        throw new UnsupportedOperationException("Unimplemented method 'reachedDist'");
    }
    
    public double getTurnEncoderPosition() {
        return turnEncoder.getPosition().getValueAsDouble()*(180/16384);
    }

    public SwerveModulePosition getPose() {
        return null;
        // return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }


    public SwerveModuleState getSwerveModuleState() {
        return null;
        // return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }

    public void setSwerveModuleState(SwerveModuleState state) {
        // state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
        // state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition())).getCos();
        driveMotor.set(state.speedMetersPerSecond);
        // turnMotor.set(-MathUtil.clamp(angleController.calculate(turnEncoder.getAbsolutePosition(), state.angle.getDegrees()) , -Constants.SwerveModuleConstants.kMaxTurningSpeed, Constants.SwerveModuleConstants.kMaxTurningSpeed));
    }

    /**
     * Stops all motors in the module
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Outputs values to dashboard
     */
    public void outputDashboard() {
        // SmartDashboard.putNumber(module+" angle", turnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(module + " turnEncoder", getTurnEncoderPosition());
    }
}