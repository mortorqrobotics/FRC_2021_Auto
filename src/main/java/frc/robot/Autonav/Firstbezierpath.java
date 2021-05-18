package frc.robot.Autonav;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Drivetrain;
import frc.robot.Utils.BezierCurve;
import frc.robot.Utils.Convert;

public class Firstbezierpath {
    boolean stopMoving = false;
    BezierCurve firstEquation;
    BezierCurve secondEquation;
    BezierCurve thirdEquation;
    BezierCurve fourthEquation;
    BezierCurve fifthEquation;
    BezierCurve sixthEquation;
    BezierCurve seventhEquation;
    State state;

    public enum State {
        FIRST_EQUATION,
        SECOND_EQUATION,
        THIRD_EQUATION,
        FOURTH_EQUATION,
        FIFTH_EQUATION,
        SIXTH_EQUATION,
        SEVENTH_EQUATION,
        STOP;
    }

    public void FirstInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(new Pose2d(new Translation2d(3.9577, 6.8776), Rotation2d.fromDegrees(0)));
        firstEquation = new BezierCurve((t) -> (39.5*Math.pow(t, 0)*Math.pow(1-t, 3)+253.5*Math.pow(t, 1)*Math.pow(1-t, 2)+354.0*Math.pow(t, 2)*Math.pow(1-t, 1)+131.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-82.0*Math.pow(t, 0)*Math.pow(1-t, 3)-201.0*Math.pow(t, 1)*Math.pow(1-t, 2)-199.5*Math.pow(t, 2)*Math.pow(1-t, 1)-82.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 9.632);
        secondEquation = new BezierCurve((t) -> (131.0*Math.pow(t, 0)*Math.pow(1-t, 3)+454.5*Math.pow(t, 1)*Math.pow(1-t, 2)+423.0*Math.pow(t, 2)*Math.pow(1-t, 1)+121.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-81.0*Math.pow(t, 0)*Math.pow(1-t, 3)-300.0*Math.pow(t, 1)*Math.pow(1-t, 2)-367.5*Math.pow(t, 2)*Math.pow(1-t, 1)-118.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 5.548);
        thirdEquation = new BezierCurve((t) -> (121.5*Math.pow(t, 0)*Math.pow(1-t, 3)+312.0*Math.pow(t, 1)*Math.pow(1-t, 2)+319.5*Math.pow(t, 2)*Math.pow(1-t, 1)+131.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-118.0*Math.pow(t, 0)*Math.pow(1-t, 3)-345.0*Math.pow(t, 1)*Math.pow(1-t, 2)-261.0*Math.pow(t, 2)*Math.pow(1-t, 1)-81.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 5.281);
        fourthEquation = new BezierCurve((t) -> (131.5*Math.pow(t, 0)*Math.pow(1-t, 3)+646.5*Math.pow(t, 1)*Math.pow(1-t, 2)+718.5*Math.pow(t, 2)*Math.pow(1-t, 1)+215.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-81.0*Math.pow(t, 0)*Math.pow(1-t, 3)-228.0*Math.pow(t, 1)*Math.pow(1-t, 2)-145.5*Math.pow(t, 2)*Math.pow(1-t, 1)-36.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 11.880);
        fifthEquation = new BezierCurve((t) -> (216.0*Math.pow(t, 0)*Math.pow(1-t, 3)+490.5*Math.pow(t, 1)*Math.pow(1-t, 2)+541.5*Math.pow(t, 2)*Math.pow(1-t, 1)+223.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-36.0*Math.pow(t, 0)*Math.pow(1-t, 3)-69.0*Math.pow(t, 1)*Math.pow(1-t, 2)-262.5*Math.pow(t, 2)*Math.pow(1-t, 1)-109.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 11.585);
        sixthEquation = new BezierCurve((t) -> (223.5*Math.pow(t, 0)*Math.pow(1-t, 3)+820.5*Math.pow(t, 1)*Math.pow(1-t, 2)+873.0*Math.pow(t, 2)*Math.pow(1-t, 1)+235.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-109.0*Math.pow(t, 0)*Math.pow(1-t, 3)-409.5*Math.pow(t, 1)*Math.pow(1-t, 2)-204.0*Math.pow(t, 2)*Math.pow(1-t, 1)-77.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 10.315);
        seventhEquation = new BezierCurve((t) -> (235.5*Math.pow(t, 0)*Math.pow(1-t, 3)+526.5*Math.pow(t, 1)*Math.pow(1-t, 2)+475.5*Math.pow(t, 2)*Math.pow(1-t, 1)+40.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , (t) -> (-77.0*Math.pow(t, 0)*Math.pow(1-t, 3)-250.5*Math.pow(t, 1)*Math.pow(1-t, 2)-150.0*Math.pow(t, 2)*Math.pow(1-t, 1)-62.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.1
                                        , 19.680);
                                
        state = State.FIRST_EQUATION;
        stopMoving = false;
    }

    public void FirstPeriodic(Drivetrain drive, Trajectory trajectory, Timer timer, RamseteController ramsete){
        double degree = GetDegree(drive, drive.distanceTravelledInMeters);
        if (!stopMoving)
            drive.drive(3.0, degree, 0);
        else{
            drive.drive(0.0, 0, 0);
        }
    }

    double x;
    double t;
    double slope;
    double angle;
    double changeInX;
    public double GetDegree(Drivetrain drive, double distance) {
        // x = drive.getPose().getX() - 2.1;

        switch (state) {
            case FIRST_EQUATION:
                if (distance > firstEquation.distance) {
                    state = State.SECOND_EQUATION;
                }

                t = distance / firstEquation.distance;
                changeInX = firstEquation.funcX.apply(t + 0.0001) - firstEquation.funcX.apply(t);
                slope = -firstEquation.derive(t);
                angle = Convert.getDegrees(slope, 1);
                break;
            case SECOND_EQUATION:
                distance -= firstEquation.distance;
                if (distance > secondEquation.distance) {
                    state = State.THIRD_EQUATION;
                }
                
                t = distance / secondEquation.distance;
                changeInX = secondEquation.funcX.apply(t + 0.0001) - secondEquation.funcX.apply(t);

                slope = -secondEquation.derive(t);
                angle =  Convert.getDegrees(slope, 1);
                break;
            case THIRD_EQUATION:
                distance -= firstEquation.distance + secondEquation.distance;
                if (distance > thirdEquation.distance) {
                    state = State.FOURTH_EQUATION;
                }

                t = distance / thirdEquation.distance;
                changeInX = thirdEquation.funcX.apply(t + 0.0001) - thirdEquation.funcX.apply(t);

                slope = -thirdEquation.derive(t);
                angle =  Convert.getDegrees(slope, 1);
                break;
            case FOURTH_EQUATION:
                distance -= firstEquation.distance + secondEquation.distance + thirdEquation.distance;
                if (distance > fourthEquation.distance) {
                    state = State.FIFTH_EQUATION;
                }

                t = distance / fourthEquation.distance;
                changeInX = fourthEquation.funcX.apply(t + 0.0001) - fourthEquation.funcX.apply(t);

                slope = -fourthEquation.derive(t);
                angle =  Convert.getDegrees(slope, 1);
                break;
            case FIFTH_EQUATION:
                distance -= firstEquation.distance + secondEquation.distance + thirdEquation.distance + fourthEquation.distance;
                if (distance > fifthEquation.distance) { 
                    state = State.SIXTH_EQUATION;
                }

                t = distance / fifthEquation.distance;
                changeInX = fifthEquation.funcX.apply(t + 0.0001) - fifthEquation.funcX.apply(t);

                slope = -fifthEquation.derive(t);
                angle =  Convert.getDegrees(slope, 1);
                break;
            case SIXTH_EQUATION:
                distance -= firstEquation.distance + secondEquation.distance + thirdEquation.distance + fourthEquation.distance + fifthEquation.distance;
                if (distance > sixthEquation.distance) { 
                    state = State.SEVENTH_EQUATION;
                }

                t = distance / sixthEquation.distance;
                changeInX = sixthEquation.funcX.apply(t + 0.0001) - sixthEquation.funcX.apply(t);
                slope = -sixthEquation.derive(t);
                angle =  Convert.getDegrees(slope, 1);
                break;
            case SEVENTH_EQUATION:
                distance -= firstEquation.distance + secondEquation.distance + thirdEquation.distance + fourthEquation.distance + fifthEquation.distance + sixthEquation.distance;
                if (distance > seventhEquation.distance) { 
                    state = State.STOP;
                }

                t = distance / seventhEquation.distance;
                changeInX = seventhEquation.funcX.apply(t + 0.0001) - seventhEquation.funcX.apply(t);
                slope = -seventhEquation.derive(t);
                angle = Convert.getDegrees(slope, 1);
                break;
            default:
                stopMoving = true;
                angle = 0.0;
                break;
        }

        if (changeInX < 0) {
            angle += 180;
        }

        return angle;
    }
}
