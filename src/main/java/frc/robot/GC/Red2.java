package frc.robot.GC;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Drivetrain;
public class Red2 {  
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
    public void changeState(States state, Drivetrain drive){
        drive.distanceTravelledInMeters = 0;
        this.state = state;
    }
    public void init(Drivetrain drive) {
        // x and y: 0.588448, 5.519395
        drive.resetOdometry(new Pose2d(new Translation2d(0.588, 5.519), Rotation2d.fromDegrees(180)));
        drive.simGyro.setHeading(Rotation2d.fromDegrees(0));


        state = States.FIRST_DRIVE;

        drive.distanceTravelledInMeters = 0;
    }
    public void periodic(Drivetrain drive) {

        switch (state) {
            case FIRST_DRIVE:
                drive.drive(2, 180, 0);

                if (drive.distanceTravelledInMeters > 3.5) {
                    changeState(States.SECOND_DRIVE, drive);

                }

                break;

            case SECOND_DRIVE:
                drive.drive(0, 0, -0.25);

                if ((drive.distanceTravelledInMeters) > (46.0/360) * 0.874) {
                    changeState(States.THIRD_DRIVE, drive);

                }
                break;

            case THIRD_DRIVE:
                if (drive.distanceTravelledInMeters> 3.9) {
                    changeState(States.FOURTH_DRIVE, drive);

                }

                drive.drive(2, 180, 0);

                break;

                case FOURTH_DRIVE:
                drive.drive(0, 0, 0.25);

                if ((drive.distanceTravelledInMeters) > (109.5/360) * 0.874) {
                    changeState(States.FIFTH_DRIVE, drive);
                }
                break;


                case FIFTH_DRIVE:
                drive.drive(2, 180, 0.00);

                if ((drive.distanceTravelledInMeters) > 4.15) {
                    changeState(States.SIXTH_DRIVE, drive);
                }
                break;

                case SIXTH_DRIVE:
                drive.drive(2, 134.3, 0);

                if ((drive.distanceTravelledInMeters) > 6.0) {
                    changeState(States.STOP, drive);

                }
                break;


            default:
                drive.drive(0, 0, 0);
                break;
        }
    }  
}
