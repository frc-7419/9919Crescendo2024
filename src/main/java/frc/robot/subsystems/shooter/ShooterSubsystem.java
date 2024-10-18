// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final VelocityVoltage velocityVoltage;

    public ShooterSubsystem() {
        this.topMotor = new TalonFX(Constants.ShooterConstants.topShooterID, Constants.RobotConstants.kCanbus);
        this.bottomMotor = new TalonFX(Constants.ShooterConstants.bottomShooterID, Constants.RobotConstants.kCanbus);
        topMotor.setInverted(true);    //TODO: when robot is built and motor are in place check if inversion is needed
        bottomMotor.setInverted(false);//TODO: when robot is built and motor are in place check if inversion is needed
        this.velocityVoltage = new VelocityVoltage(0).withSlot(0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        config.Slot0.kS = 0.1; // To account for friction                                                     TODO: calculate friction
        config.Slot0.kV = 0.11299435; // volts per rotation per second                                        DONT TOUCH
        config.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output                 TODO: tune value for P
        config.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
        config.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
        config.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        topMotor.getConfigurator().apply(config);
        bottomMotor.getConfigurator().apply(config);
    }

    /**
     * Runs the motors of the shooter at the given speeds in RPS
     *
     * @param bottomSpeed speed of the bottom motor in RPS, positive will push the note forward
     * @param topSpeed    speed of the top motor in RPS, positive will push the note forward
     */
    public void run(final double topRPM, final double bottomRPM) {
        if (topRPM != 0 || bottomRPM != 0) {
            coast();
            topMotor.setControl(velocityVoltage.withVelocity(topRPM / 60.0D));
            bottomMotor.setControl(velocityVoltage.withVelocity(bottomRPM / 60.0D));
        } else {
            brake();
            topMotor.setControl(velocityVoltage.withVelocity(topRPM / 60.0D));
            bottomMotor.setControl(velocityVoltage.withVelocity(bottomRPM / 60.0D));
        }
    }

    /**
     * Gets the velocity of the top motor
     *
     * @return velocity in RPS, positive means forward
     */
    public double getTopVelocity() {
        return topMotor.getVelocity().getValue();
    }

    /**
     * Gets the velocity of the bottom motor
     *
     * @return velocity in RPS, positive means forward
     */
    public double getBottomVelocity() {
        return bottomMotor.getVelocity().getValue();
    }

    public void stop(){
        topMotor.setControl(velocityVoltage.withVelocity(0));
        bottomMotor.setControl(velocityVoltage.withVelocity(0));
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/Top Velocity", getTopVelocity());
        SmartDashboard.putNumber("shooter/Bottom Velocity", getBottomVelocity());
        SmartDashboard.putNumber("shooter/Top Voltage: ", topMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("shooter/Bottom Voltage: ", bottomMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("shooter/Top Temperature ", topMotor.getDeviceTemp().getValue());
        SmartDashboard.putNumber("shooter/Bottom Temperature", bottomMotor.getDeviceTemp().getValue());
    }

    void coast() {
        topMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    void brake() {
        topMotor.setNeutralMode(NeutralModeValue.Brake);
        bottomMotor.setNeutralMode(NeutralModeValue.Brake);
    }
}
