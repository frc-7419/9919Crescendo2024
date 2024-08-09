// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.haptics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Haptics extends SubsystemBase {
  /** Creates a new Haptics. */
  public enum ControllerType {
    DRIVER,
    OPERATOR,
    BOTH
  }

  public enum RumbleMode {
    NONE,
    RUMBLE,
    WARN
  }

  private XboxController driverHID;
  private XboxController operatorHID;

  private RumbleMode currentMode = RumbleMode.NONE;

  private double rumbleStartTime = 0;
  private double rumbleDuration = 0;
  private double pulseInterval = 0;
  private double pulseEndTime = 0;

  private HapticRequest currentRequest;

  private Timer masterTimer = new Timer();

  public Haptics(CommandXboxController driver, CommandXboxController operator) {
    driverHID = driver.getHID();
    operatorHID = operator.getHID();
    masterTimer.start();
  }

  private double getTime() {
    return masterTimer.get();
  }

  public void startRumble(ControllerType controller, double intensity) {
    switch (controller) {
      case DRIVER:
        driverHID.setRumble(XboxController.RumbleType.kRightRumble, intensity);
        driverHID.setRumble(XboxController.RumbleType.kLeftRumble, intensity);
        break;
      case OPERATOR:
        operatorHID.setRumble(XboxController.RumbleType.kRightRumble, intensity);
        operatorHID.setRumble(XboxController.RumbleType.kLeftRumble, intensity);
        break;
      case BOTH:
        startRumble(ControllerType.DRIVER, intensity);
        startRumble(ControllerType.OPERATOR, intensity);
        break;
    }
  }

  public void stopRumble(ControllerType controller) {
    startRumble(controller, 0.0);
  }

  public void setRequest(HapticRequest request) {
    currentRequest = request;
    switch (request.getRequestType()) {
      case RUMBLE:
        rumbleForDuration(request.getControllerType(), request.getIntensity(), request.getDuration());
        break;
      case WARN:
        warningRumble(request.getControllerType(), request.getIntensity(), request.getInterval(),
            request.getDuration());
      case STOP:
        stopRumble(request.getControllerType());
        break;
    }
  }

  public void rumbleForDuration(ControllerType controller, double intensity, double duration) {
    currentMode = RumbleMode.RUMBLE;
    rumbleStartTime = getTime();
    rumbleDuration = duration;
    startRumble(controller, intensity);
  }

  public void warningRumble(ControllerType controller, double intensity, double interval, double duration) {
    pulseInterval = interval;
    rumbleStartTime = getTime();
    pulseEndTime = rumbleStartTime + duration;
    currentMode = RumbleMode.WARN;
  }

  @Override
  public void periodic() {
    double currentTime = getTime();

    switch (currentMode) {
      case RUMBLE:
        if (currentTime - rumbleStartTime >= rumbleDuration) {
          stopRumble(currentRequest.getControllerType());
          currentMode = RumbleMode.NONE;
        }
        break;
      case WARN:
        if (currentTime >= pulseEndTime) {
          stopRumble(currentRequest.getControllerType());
          currentMode = RumbleMode.NONE;
        } else if ((int) ((currentTime - rumbleStartTime) / pulseInterval) % 2 == 0) {
          startRumble(currentRequest.getControllerType(), currentRequest.getIntensity());
        } else {
          stopRumble(currentRequest.getControllerType());
        }
        break;
      case NONE:
      default:
        break;
    }
  }
}
