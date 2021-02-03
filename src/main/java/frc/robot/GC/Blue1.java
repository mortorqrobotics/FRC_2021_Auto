package frc.robot.GC;

import java.time.Duration;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Drivetrain;

public class Blue1 {
    public enum States {
        FIRST_DRIVE,
        SECOND_DRIVE,
        THIRD_DRIVE,
        FOURTH_DRIVE,
        FIFTH_DRIVE,
        SIXTH_DRIVE,
        STOP;
    }

    States state;

    public void init(Drivetrain drive) {
        // x and y: 0.588448, 4.097343
        drive.resetOdometry(new Pose2d(new Translation2d(0.517752, 1.325445), Rotation2d.fromDegrees(180)));
        drive.simGyro.setHeading(Rotation2d.fromDegrees(0));

        state = States.FIRST_DRIVE;

        drive.distanceTravelledInMeters = 0;
    }

    private void changeState(States state, Drivetrain drive) {
        this.state = state;
        drive.distanceTravelledInMeters = 0.0;
    }

    public void periodic(Drivetrain drive) {

        switch (state) {
            case FIRST_DRIVE:
                drive.drive(2, 180, 0);

                if (drive.distanceTravelledInMeters > 7.5) {
                    changeState(States.SECOND_DRIVE, drive);
                }

                break;

            case SECOND_DRIVE:
                drive.drive(0, 0, 0.2);

                if (drive.distanceTravelledInMeters > (71.6/360) * 0.874) {
                    changeState(States.THIRD_DRIVE, drive);
                }

                break;

            case THIRD_DRIVE:
                if (drive.distanceTravelledInMeters > 4.4) {

                    changeState(States.FOURTH_DRIVE, drive);
                }

                drive.drive(2, 180, 0);

                break;
            
            case FOURTH_DRIVE:
                drive.drive(0, 0, -0.2);

                if (drive.distanceTravelledInMeters > ((117.9/360) * 0.874)) {
                    changeState(States.FIFTH_DRIVE, drive);
                }

                break;

            case FIFTH_DRIVE:
                drive.drive(2, 180, 0);

                if (drive.distanceTravelledInMeters > 4.3) {
                    changeState(States.SIXTH_DRIVE, drive);
                }

                break;

            case SIXTH_DRIVE:
                drive.drive(2, -drive.getPose().getRotation().getDegrees(), 0);

                if (drive.getPose().getX() > 15.5) {
                    state = States.STOP;
                }

                break;
            default:
                drive.drive(0, 0, 0);
                break;
        }
    }
}
