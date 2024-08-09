package frc.robot.subsystems.haptics;

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

    private static final double DEFAULT_INTENSITY = 0.7;
    private static final double DEFAULT_PULSE_INTERVAL = 0.5;

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
            double duration, double interval, double intensity) {
        this.controllerType = controllerType;
        this.requestType = requestType;
        this.duration = duration;
        this.interval = interval;
        this.intensity = intensity;
    }

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType, double duration,
            double interval) {
        this(controllerType, requestType, duration, interval, DEFAULT_INTENSITY);
    }

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType,
            double duration) {
        this(controllerType, requestType, duration, DEFAULT_PULSE_INTERVAL, DEFAULT_INTENSITY);
    }

    public HapticRequest(Haptics.ControllerType controllerType, RequestType requestType) {
        this(controllerType, requestType, 0, 0, 0);
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

    public void dump() {
        System.out.println("Controller: " + controllerType + " Request: " + requestType + " Intensity: " + intensity
                + " Duration: " + duration + " Interval: " + interval);
    }
}
