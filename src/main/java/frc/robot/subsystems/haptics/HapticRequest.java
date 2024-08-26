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

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
                         double duration, double interval, double intensity, boolean debug) {
        this.controllerType = controllerType;
        this.requestType = requestType;
        this.duration = duration;
        this.interval = interval;
        this.intensity = intensity;
        this.debug = debug;
    }

    /* 
     * Constructor for a haptic request with a specified controller type, request type, duration, interval, and intensity.
     * @param controllerType the controller type to send the request to
     * @param requestType the type of request to send
     * @param duration the duration of the request
     * @param interval the interval between pulses
     * @param intensity the intensity of the request
    */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
                         double interval, double intensity) {
        this(controllerType, requestType, duration, interval, intensity, false);
    }

    /*
     * Constructor for a haptic request with a specified controller type, request type, duration, and interval.
     * @param controllerType the controller type to send the request to
     * @param requestType the type of request to send
     * @param duration the duration of the request
     * @param interval the interval between pulses
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
                         double interval) {
        this(controllerType, requestType, duration, interval, DEFAULT_INTENSITY);
    }

    /*
     * Constructor for a haptic request with a specified controller type, request type, and duration.
     * @param controllerType the controller type to send the request to
     * @param requestType the type of request to send
     * @param duration the duration of the request
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
                         double duration) {
        this(controllerType, requestType, duration, DEFAULT_PULSE_INTERVAL);
    }

    /*
     * Constructor for a haptic request with a specified controller type and request type.
     * @param controllerType the controller type to send the request to
     * @param requestType the type of request to send
     */
    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType) {
        this(controllerType, requestType, Integer.MAX_VALUE);
    }

    /**Constructor for a haptic request with a specified intensity (how much it rumbles you know what im sayin) */
    public HapticRequest withIntensity(double intensity) {
        this.intensity = intensity;
        return this;
    }
    
    /*
     * Sets the controller type of the haptic request.
     * @param requestType the type of request to send
     */
    public HapticRequest withType(RequestType requestType) {
        this.requestType = requestType;
        return this;
    }

    /*
     * Sets the duration of the haptic request.
     * @param duration the duration of the request
     */
    public HapticRequest withDuration(double duration) {
        this.duration = duration;
        return this;
    }

    /*
     * Sets the interval of the haptic request.
     * @param interval the interval between pulses
     */
    public HapticRequest withInterval(double interval) {
        this.interval = interval;
        return this;
    }

    /*
     * Sets the debug mode of the haptic request.
     * @param debug the debug mode of the request
     */
    public HapticRequest withDebug(boolean debug) {
        this.debug = debug;
        return this;
    }

    /*
     * Starts the request.
     * @param currentTime the current time
     */
    public void startRequest(double currentTime) {
        this.startTime = currentTime;
        this.endTime = currentTime + duration;
        this.active = true;
    }

    /*
     * Checks if the request is active.
     * @param currentTime the current time
     * @return true if the request is active, false otherwise
     */
    public boolean isActive(double currentTime) {
        return active && currentTime < endTime;
    }
    
    /*
     * Processes the request.
     * @param haptics the haptics subsystem
     * @param currentTime the current time
     */
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
    /*
     * Get the controller type of the haptic request.
     * @return the controller type of the request
     */
    public Haptics.ControllerType getControllerType() {
        return controllerType;
    }
    /*
     * Get the request type of the haptic request.
     * @return the request type of the request
     */
    public RequestType getRequestType() {
        return requestType;
    }
    /*
     * Get the intensity of the haptic request.
     * @return the intensity of the request
     */
    public double getIntensity() {
        return intensity;
    }
    /*
     * Get the duration of the haptic request.
     * @return the duration of the request
     */
    public double getDuration() {
        return duration;
    }
    /*
     * Get the interval of the haptic request.
     * @return the interval of the request
     */
    public double getInterval() {
        return interval;
    }

    /*
     * Get the start time of the haptic request.
     */
    private void debug(String message) {
        if (debug) {
            System.out.println(message);
        }
    }

    /*
     * Dumps the haptic request.
     */
    public String dump() {
        return ("Active: " + active + " Controller: " + controllerType + " Request: " + requestType
                + " Intensity: " + intensity
                + " Duration: " + duration + " Interval: " + interval);
    }
    /*
     * The type of request.
     */
    public enum RequestType {
        RUMBLE,
        WARN,
        STOP
    }
}
