package frc.robot.Utils;

import java.util.function.DoubleFunction;

public class BezierCurve {
    public DoubleFunction<Double> funcX;
    public DoubleFunction<Double> funcY;
    public double distance;
    private double DT = 0.001;

    public BezierCurve(DoubleFunction<Double> funcX, DoubleFunction<Double> funcY, double distance) {
        this.funcX = funcX;
        this.funcY = funcY;
        this.distance = distance;
    }

    public double derive(double t) {
        return (funcY.apply(t + DT) - funcY.apply(t)) / (funcX.apply(t + DT) - funcX.apply(t));
    }
}
