package frc.robot.Utils;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class ConvertToDegrees {
    
    public static double getDegrees(double x, double y) {
        double degrees = 0;

        if (x == 0) {
            degrees = (y < 0) ? 270 : 90;
        } else {
            degrees = Math.toDegrees(Math.atan(y / x));

            if (y == 0) {
                degrees = (x < 0) ? 180 : 0;
            }
        }

        if (x < 0) {
            degrees += 180;
        } else if (y < 0) {
            degrees += 360;
        }

        degrees -= 90;

        return degrees;
    }

}
