// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.handoff;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandoffConstants;


public class HandoffSubsystem extends SubsystemBase {
    /*
     * Just a diverter wheel powered by a rev.
     *
     * Clockwise: To Elevator
     * Counter-Clockwise: To Intake
     */
      
    private final CANSparkMax handoffMotor;

    public HandoffSubsystem() {
        handoffMotor = new CANSparkMax(HandoffConstants.loaderID, MotorType.kBrushless);
        handoffMotor.setInverted(true);
        coast();
    }

    // Method to apply power to the handoff motor for the command.

    // The other basic motor control methods. Eg coast, brake, etc.

    /*Setting the handoff motors to neutral mode. */
    public void coast() {
        handoffMotor.setIdleMode(IdleMode.kCoast);
    }
    /*Setting the handoff motors to brake mode. */
    public void brake() {
        handoffMotor.setIdleMode(IdleMode.kCoast);
    }
    /*Setting the handoff motors to stop. */
    public void stop() {
        handoffMotor.set(0);
    }
/*Outputitng the voltage of the handoff motor*/
    public void run(final double percent) {
        handoffMotor.set(percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
