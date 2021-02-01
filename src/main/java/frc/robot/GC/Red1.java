package frc.robot.GC;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Drivetrain;

public class Red1 {
    
    public enum States {
        FIRST_DRIVE,
        SECOND_DRIVE,
        THIRD_DRIVE,
        STOP;
    }

    States state;

    public void init(Drivetrain drive) {
        // x and y: 0.588448, 4.097343
        drive.resetOdometry(new Pose2d(new Translation2d(0.58, 4.09), Rotation2d.fromDegrees(180)));
        drive.simGyro.setHeading(Rotation2d.fromDegrees(0));

        state = States.FIRST_DRIVE;

        drive.distanceTravelledInMeters = 0;
    }

    public void periodic(Drivetrain drive) {

        switch (state) {
            case FIRST_DRIVE:
                drive.drive(2, 180, 0);

                if (drive.distanceTravelledInMeters > 3.4) {
                    state = States.SECOND_DRIVE;
                }

                break;

            case SECOND_DRIVE:
                drive.drive(0, 0, -0.05);

                if ((drive.distanceTravelledInMeters - 3.4) > (31.6/360) * 0.874) {
                    state = States.THIRD_DRIVE;
                }
                break;

            case THIRD_DRIVE:
                if (drive.distanceTravelledInMeters - 3.4 - (31.6/360) * 0.874 > 3.4) {
                    state = States.STOP;
                }

                drive.drive(2, 180, 0);

                break;

            default:
                drive.drive(0, 0, 0);
                break;
        }
    }

}
