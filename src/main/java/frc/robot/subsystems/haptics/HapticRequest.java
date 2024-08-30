package frc.robot.subsystems.haptics;

import frc.robot.Constants.ControllerConstants;

public class HapticRequest {
    private static final double DEFAULT_INTENSITY = ControllerConstants.defaultRumbleIntensity;
    private static final double DEFAULT_PULSE_INTERVAL = ControllerConstants.defaultPulseInterval;
    private Haptics.ControllerType controllerType;
    private RequestType requestType;
    private double intensity;
    private double duration;
    private double interval;
    private double startTime;
    private double endTime;
    private boolean active;
    private boolean debug;
    /**
     * Initializes all the instance fields of the Haptics class
     * @param controllerType is the controller(driver or operator) that will vibrate
     * @param requestType is the type of vibration that will be used
     * @param duration is the duration of the vibration
     * @param interval is the interval between the vibrations
     * @param intensity is the intensity of the vibration
     * @param debug is the boolean to see if the vibration is active or not
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
                         double duration, double interval, double intensity, boolean debug) {
        this.controllerType = controllerType;
        this.requestType = requestType;
        this.duration = duration;
        this.interval = interval;
        this.intensity = intensity;
        this.debug = debug;
    }
    /**
     * Initializes all the instance fields of the Haptics class
     * @param controllerType is the controller(driver or operator) that will vibrate
     * @param requestType is the type of vibration that will be used
     * @param duration is the duration of the vibration
     * @param interval is the interval between the vibrations
     * @param intensity is the intensity of the vibration
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
                         double interval, double intensity) {
        this(controllerType, requestType, duration, interval, intensity, false);
    }
    /**
     * Initializes all the instance fields of the Haptics class
     * @param controllerType is the controller(driver or operator) that will vibrate
     * @param requestType is the type of vibration that will be used
     * @param duration is the duration of the vibration
     * @param interval is the interval between the vibrations
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
                         double interval) {
        this(controllerType, requestType, duration, interval, DEFAULT_INTENSITY);
    }
    /**
     * Initializes all the instance fields of the Haptics class
     * @param controllerType is the controller(driver or operator) that will vibrate
     * @param requestType is the type of vibration that will be used
     * @param duration is the duration of the vibration

     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
                         double duration) {
        this(controllerType, requestType, duration, DEFAULT_PULSE_INTERVAL);
    }
    /**
     * Initializes all the instance fields of the Haptics class
     * @param controllerType is the controller(driver or operator) that will vibrate
     * @param requestType is the type of vibration that will be used
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType) {
        this(controllerType, requestType, Integer.MAX_VALUE);
    }
    /**
     * Returns the intensity of the vibration itself
     */
    public HapticRequest withIntensity(double intensity) {
        this.intensity = intensity;
        return this;
    }
    /**
     * Returns the requestType itself
     * @return the hapticRequest object itself
     */
    public HapticRequest withType(RequestType requestType) {
        this.requestType = requestType;
        return this;
    }
    /**
     * Returns the withDuration itself
     * @return the withDuration object itself
     */
    public HapticRequest withDuration(double duration) {
        this.duration = duration;
        return this;
    }
     /**
      * Sets the interval between the vibrations
     * @param interval is the interval between the vibrations
     * @return the HapticsRequest object itself
     */
    public HapticRequest withInterval(double interval) {
        this.interval = interval;
        return this;
    }
    /**
     * Returns the withDebug itself
     * @return the withDebug object itself
     */
    public HapticRequest withDebug(boolean debug) {
        this.debug = debug;
        return this;
    }
    /**
     * Returns the startRequest itself
     *  the startRequest object itself
     */
    public void startRequest(double currentTime) {
        this.startTime = currentTime;
        this.endTime = currentTime + duration;
        this.active = true;
    }

    public boolean isActive(double currentTime) {
        return active && currentTime < endTime;
    }

    public void processRequest(Haptics haptics, double currentTime) {
        if (!isActive(currentTime)) {
            haptics.stopRumble(controllerType);
            active = false;
            return;
        }

        switch (requestType) {
            case RUMBLE:
                haptics.startRumble(controllerType, intensity);
                debug("Rumble " + dump());
                break;
            case WARN:
                if ((int) ((currentTime - startTime) / interval) % 2 == 0) {
                    haptics.startRumble(controllerType, intensity);
                    debug("Warn (ON) " + dump());
                } else {
                    haptics.stopRumble(controllerType);
                    debug("Warn (OFF) " + dump());
                }
                break;
            case STOP:
                haptics.stopRumble(controllerType);
                debug(dump());
                active = false;
                break;
        }
    }

    public Haptics.ControllerType getControllerType() {
        return controllerType;
    }

    public RequestType getRequestType() {
        return requestType;
    }

    public double getIntensity() {
        return intensity;
    }

    public double getDuration() {
        return duration;
    }

    public double getInterval() {
        return interval;
    }

    private void debug(String message) {
        if (debug) {
            System.out.println(message);
        }
    }

    public String dump() {
        return ("Active: " + active + " Controller: " + controllerType + " Request: " + requestType
                + " Intensity: " + intensity
                + " Duration: " + duration + " Interval: " + interval);
    }

    public enum RequestType {
        RUMBLE,
        WARN,
        STOP
    }
}
