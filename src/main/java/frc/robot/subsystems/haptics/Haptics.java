package frc.robot.subsystems.haptics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Haptics extends SubsystemBase {
    private XboxController driverHID;
    private XboxController operatorHID;
    private HapticRequest driverRequest;
    private HapticRequest operatorRequest;
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
        switch (request.getControllerType()) {
            case DRIVER:
                driverRequest = request;
                driverRequest.startRequest(getTime());
                break;
            case OPERATOR:
                operatorRequest = request;
                operatorRequest.startRequest(getTime());
                break;
            case BOTH:
                setRequest(new HapticRequest(ControllerType.DRIVER, request.getRequestType(), request.getDuration(),
                        request.getInterval(), request.getIntensity()));
                setRequest(new HapticRequest(ControllerType.OPERATOR, request.getRequestType(), request.getDuration(),
                        request.getInterval(), request.getIntensity()));
                break;
        }
    }

    @Override
    public void periodic() {
        double currentTime = getTime();

        if (driverRequest != null) {
            driverRequest.processRequest(this, currentTime);
        }

        if (operatorRequest != null) {
            operatorRequest.processRequest(this, currentTime);
        }
    }

    public enum ControllerType {
        DRIVER,
        OPERATOR,
        BOTH
    }
}
