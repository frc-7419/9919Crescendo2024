package frc.robot.subsystems.haptics;

import frc.robot.Constants.ControllerConstants;

public class HapticRequest {
    public enum RequestType {
        RUMBLE,
        WARN,
        STOP
    }

    private Haptics.ControllerType controllerType;
    private RequestType requestType;
    private double intensity;
    private double duration;
    private double interval;

    private double startTime;
    private double endTime;
    private boolean active;

    private static final double DEFAULT_INTENSITY = ControllerConstants.defaultRumbleIntensity;
    private static final double DEFAULT_PULSE_INTERVAL = ControllerConstants.defaultPulseInterval;

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

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
            double interval, double intensity) {
        this(controllerType, requestType, duration, interval, intensity, false);
    }

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
            double interval) {
        this(controllerType, requestType, duration, interval, DEFAULT_INTENSITY);
    }

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
            double duration) {
        this(controllerType, requestType, duration, DEFAULT_PULSE_INTERVAL);
    }

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType) {
        this(controllerType, requestType, Integer.MAX_VALUE);
    }

    public HapticRequest withIntensity(double intensity) {
        this.intensity = intensity;
        return this;
    }

    public HapticRequest withType(RequestType requestType) {
        this.requestType = requestType;
        return this;
    }

    public HapticRequest withDuration(double duration) {
        this.duration = duration;
        return this;
    }

    public HapticRequest withInterval(double interval) {
        this.interval = interval;
        return this;
    }

    public HapticRequest withDebug(boolean debug) {
        this.debug = debug;
        return this;
    }

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
}
