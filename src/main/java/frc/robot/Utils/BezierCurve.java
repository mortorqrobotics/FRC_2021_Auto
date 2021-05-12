package frc.robot.Utils;

import java.util.function.DoubleFunction;

public class BezierCurve {
    public DoubleFunction<Double> funcX;
    public DoubleFunction<Double> funcY;
    public double distance;

    public BezierCurve(DoubleFunction<Double> funcX, DoubleFunction<Double> funcY, double distance) {
        this.funcX = funcX;
        this.funcY = funcY;
        this.distance = distance;
    }
}
