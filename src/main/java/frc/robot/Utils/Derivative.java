package frc.robot.Utils;

import java.util.function.DoubleFunction;

public class Derivative {
    private static final double DX = 0.0001;

    public static DoubleFunction<Double> derive(DoubleFunction<Double> f) {
        return (x) -> (f.apply(x + DX) - f.apply(x)) / DX;
    }
}
