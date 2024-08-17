// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSubsystem extends SubsystemBase {
    private final TalonFX noteMotor;
    private final TalonFX angleMotor;
    private final VelocityVoltage velocityVoltage;
    private final MotionMagicVoltage mmVoltage;

    public AmpSubsystem() {
        this.noteMotor = new TalonFX(Constants.AmpConstants.topShooterID, Constants.RobotConstants.kCanbus);
        this.angleMotor = new TalonFX(Constants.AmpConstants.bottomShooterID, Constants.RobotConstants.kCanbus);
        noteMotor.setInverted(false);  //TODO: when robot is built and motor are in place check if inversion is needed
        angleMotor.setInverted(false); //TODO: when robot is built and motor are in place check if inversion is needed
        this.velocityVoltage = new VelocityVoltage(0).withSlot(0);
        this.mmVoltage = new MotionMagicVoltage(0).withSlot(0);

        TalonFXConfiguration vvConfig = new TalonFXConfiguration();
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        vvConfig.Slot0.kS = 0.1; // To account for friction                                                     TODO: calculate friction
        vvConfig.Slot0.kV = 0.11299435; // volts per rotation per second                                        DONT TOUCH
        vvConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 TODO: tune value for P
        vvConfig.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
        vvConfig.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
        vvConfig.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
        vvConfig.Voltage.PeakForwardVoltage = 12;
        vvConfig.Voltage.PeakReverseVoltage = -12;

        TalonFXConfiguration mmConfig = new TalonFXConfiguration();
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        mmConfig.Slot0.kS = 0.1; // Friction                                                                    TODO: Friction, tune kS
        mmConfig.Slot0.kV = 0.11299435; // volts per rotation per second                                        TODO: tune kV
        mmConfig.Slot0.kG = 0; // Gravity                                                                       TODO: figure out gravity
        mmConfig.Slot0.kP = 0.11; // Dependent on position                                                      TODO: tune P
        mmConfig.Slot0.kI = 0; // Dependent on accumulated error                                                TODO: tune I
        mmConfig.Slot0.kD = 0; // Dependent on speed                                                            TODO: tune D

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfig = mmConfig.MotionMagic;
        motionMagicConfig.MotionMagicCruiseVelocity = 80; // rps cruise velocity TODO
        motionMagicConfig.MotionMagicAcceleration = 160; // rps/s acceleration TODO
        motionMagicConfig.MotionMagicJerk = 1600; // rps/s^2 jerk TODO

        noteMotor.getConfigurator().apply(vvConfig);
        angleMotor.getConfigurator().apply(mmConfig);
        noteMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs the motors of the amp shooter at the given speeds in RPS
     *
     * @param bottomSpeed speed of the bottom motor in RPS, positive will push the note forward
     * @param topSpeed    speed of the top motor in RPS, positive will push the note forward
     */
    public void run(final double RPM) {
        if (RPM != 0) {
            coast();
            noteMotor.setControl(velocityVoltage.withVelocity(RPM / 60.0D));
        } else {
            brake();
            noteMotor.setControl(velocityVoltage.withVelocity(RPM));
        }
    }

    /**
     * position is in rotations
     *
     * @param position
     */
    public void goToPosition(final double position) {
        angleMotor.setControl(mmVoltage.withPosition(position));
    }

    /**
     * Gets the velocity of the top motor
     *
     * @return velocity in RPS, positive means forward
     */
    public double getTopVelocity() {
        return noteMotor.getVelocity().getValue();
    }

    /**
     * Gets the position of the angle motor
     *
     * @return position in rotations, positive means forward
     */
    public double getAnglePosition() {
        return angleMotor.getPosition().getValue();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("amp/Top Velocity", getTopVelocity());
        SmartDashboard.putNumber("amp/Angle Position", getAnglePosition());
        SmartDashboard.putNumber("amp/Top Voltage", noteMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("amp/Angle Voltage", angleMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("amp/Top Temperature", noteMotor.getDeviceTemp().getValue());
        SmartDashboard.putNumber("amp/Angle Temperature", angleMotor.getDeviceTemp().getValue());
    }

    private void coast() {
        noteMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void brake() {
        noteMotor.setNeutralMode(NeutralModeValue.Brake);
    }
}
