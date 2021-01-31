package frc.robot.GC;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Drivetrain;

public class Red1 {
    
    public void init(Drivetrain drive) {
        // x and y: 0.588448, 4.097343
        drive.resetOdometry(new Pose2d(new Translation2d(0.58, 4.09), Rotation2d.fromDegrees(180)));
        drive.simGyro.setHeading(Rotation2d.fromDegrees(0));
    }

    public void periodic(Drivetrain drive) {

        if (drive.distanceTravelledInMeters < 3.4) {
            drive.drive(2, 180, 0);
        }

        else if (drive.distanceTravelledInMeters - 3.4 < 31.6/360 * .874) {
            drive.drive(0, 0, 0.2);
        }

        else {
            drive.drive(0, 0, 0);
        }
    }



}
