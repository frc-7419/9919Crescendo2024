// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private final VelocityVoltage velocityVoltage;
    private final VoltageOut voltReq = new VoltageOut(0.0);
    /* Use one of these sysidroutines for your particular test */
    private final SysIdRoutine SysIdRoutineFlywheel;

    public ShooterSubsystem() {
        this.topMotor = new TalonFX(Constants.ShooterConstants.topShooterID, "9919");
        this.bottomMotor = new TalonFX(Constants.ShooterConstants.bottomShooterID, "9919");
        this.velocityVoltage = new VelocityVoltage(0).withSlot(0);
        SysIdRoutineFlywheel = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> topMotor.setControl(voltReq.withOutput(volts.in(Volts))),
                    null,
                    this));
        TalonFXConfiguration config = new TalonFXConfiguration();
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        config.Slot0.kS = 0.18239; // To account for friction                                                     TODO: calculate friction
        config.Slot0.kV = 0.11451; // volts per rotation per second                                        DONT TOUCH
        config.Slot0.kP = 0.029207; // An error of 1 rotation per second results in 0.11 V output                 TODO: tune value for P
        config.Slot0.kI = 0; // No output for integrated error                                                DONT TOUCH
        config.Slot0.kD = 0; // No output for error derivative                                                DONT TOUCH
        config.Slot0.kG = 0; // No gravity :)                                                                 DONT TOUCH
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        topMotor.getConfigurator().apply(config);
        bottomMotor.getConfigurator().apply(config);
        topMotor.setInverted(false);   
        bottomMotor.setInverted(true);
    }

    /**
     * Runs the motors of the shooter at the given speeds in RPM
     *
     * @param bottomSpeed speed of the bottom motor in RPM, positive will push the note forward
     * @param topSpeed    speed of the top motor in RPM, positive will push the note forward
     */
    public void run(final double topRPM, final double bottomRPM) {
        if (topRPM != 0 && bottomRPM != 0) {
            coast();
            topMotor.setControl(velocityVoltage.withAcceleration(0.01).withVelocity(topRPM / 60.0D));
            bottomMotor.setControl(velocityVoltage.withAcceleration(0.01).withVelocity(bottomRPM / 60.0D));
        } else {
            topMotor.setControl(velocityVoltage.withVelocity(topRPM));
            bottomMotor.setControl(velocityVoltage.withVelocity(bottomRPM));
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


    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/Top Velocity", getTopVelocity());
        SmartDashboard.putNumber("shooter/Bottom Velocity", getBottomVelocity());
        SmartDashboard.putNumber("shooter/Top Voltage: ", topMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("shooter/Bottom Voltage: ", bottomMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("shooter/Top Temperature ", topMotor.getDeviceTemp().getValue());
        SmartDashboard.putNumber("shooter/Bottom Temperature", bottomMotor.getDeviceTemp().getValue());
    }

    private void coast() {
        topMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void brake() {
        topMotor.setNeutralMode(NeutralModeValue.Brake);
        bottomMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return SysIdRoutineFlywheel.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return SysIdRoutineFlywheel.dynamic(direction);
    }
}
