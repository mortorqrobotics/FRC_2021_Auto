package frc.robot.Utils;

import java.util.function.DoubleFunction;

public class BezierDerivative {
    private static final double DT = 0.0001;
    
    public static double derive(DoubleFunction<Double> fx, DoubleFunction<Double> fy, double t) {
        return (fy.apply(t + DT) - fy.apply(t)) / (fx.apply(t + DT) - fx.apply(t));
    }
}
