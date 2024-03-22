package team4384.robot.constants;

public class PidConfig {
    public double p;
    public double i;
    public double d;
    public double iZone;
    public double ff;
    public double peakOutput;

    public PidConfig(double p, double i, double d, double ff, double iZone, double peakOutput) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ff = ff;
        this.iZone = iZone;
        this.peakOutput = peakOutput;
    }
}
