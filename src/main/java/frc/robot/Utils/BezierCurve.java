package frc.robot.Utils;

import java.util.function.DoubleFunction;

public class BezierCurve {
    public DoubleFunction<Double> funcX;
    public DoubleFunction<Double> funcY;
    public double distance;
    private double DT = 0.0001;

    public BezierCurve(DoubleFunction<Double> funcX, DoubleFunction<Double> funcY) {
        this.funcX = funcX;
        this.funcY = funcY;

        double length = 0;
        for(double t = 0; t < 1; t += DT) {
            length += Math.sqrt(
                Math.pow((funcX.apply(t + DT) - funcX.apply(t)), 2)
                + Math.pow((funcY.apply(t + DT) - funcY.apply(t)), 2)
            );
        }
        this.distance = length;
    }

    public double derive(double t) {
        return (funcY.apply(t + DT) - funcY.apply(t)) / (funcX.apply(t + DT) - funcX.apply(t));
    }
}
