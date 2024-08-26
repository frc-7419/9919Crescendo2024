package frc.robot.subsystems.haptics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Haptics extends SubsystemBase {
    // Instance Fields
    private XboxController driverHID;
    private XboxController operatorHID;
    private HapticRequest driverRequest;
    private HapticRequest operatorRequest;
    private Timer masterTimer = new Timer();

    /*
     * Constructor for the Haptics subsystem.
     * @param driver the driver's controller
     * @param operator the operator's controller
     */
    public Haptics(CommandXboxController driver, CommandXboxController operator) {
        driverHID = driver.getHID();
        operatorHID = operator.getHID();
        masterTimer.start();
    }
    /*
     * Gets the current time from the master timer.
     * @return the current time
     */
    private double getTime() {
        return masterTimer.get();
    }

    /*
     * Starts the rumble on the specified controller.
     * @param controller the controller to start the rumble on
     * @param intensity the intensity of the rumble
     */
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

    /*
     * Stops the rumble on the specified controller.
     * @param controller the controller to stop the rumble on
     */
    public void stopRumble(ControllerType controller) {
        startRumble(controller, 0.0);
    }
    
    /*
     * Sets the request for the specified controller.
     * @param request the haptic request
     */
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
    /*
     * Periodic method for the Haptics subsystem.
     */
    public void periodic() {
        double currentTime = getTime();

        if (driverRequest != null) {
            driverRequest.processRequest(this, currentTime);
        }

        if (operatorRequest != null) {
            operatorRequest.processRequest(this, currentTime);
        }
    }
    // The ControllerType can be set to any three of these constants.
    public enum ControllerType {
        DRIVER,
        OPERATOR,
        BOTH
    }
}
