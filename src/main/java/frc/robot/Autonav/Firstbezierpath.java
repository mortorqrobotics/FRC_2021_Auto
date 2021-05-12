package frc.robot.Autonav;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Drivetrain;
import frc.robot.Utils.BezierCurve;
import frc.robot.Utils.BezierDerivative;
import frc.robot.Utils.Convert;

public class Firstbezierpath {
    boolean stopMoving = false;
    BezierCurve firstEquation;
    BezierCurve secondEquation;
    BezierCurve thirdEquation;
    State state;

    public enum State {
        FIRST_EQUATION,
        SECOND_EQUATION,
        THIRD_EQUATION,
        STOP;
    }

    public void FirstInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(trajectory.getInitialPose());
        firstEquation = new BezierCurve((t) -> (57.0*Math.pow(t, 0)*Math.pow(1-t, 3)+364.5*Math.pow(t, 1)*Math.pow(1-t, 2)+486.0*Math.pow(t, 2)*Math.pow(1-t, 1)+189.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.03686
                                        , (t) -> (-118.5*Math.pow(t, 0)*Math.pow(1-t, 3)-289.5*Math.pow(t, 1)*Math.pow(1-t, 2)-292.5*Math.pow(t, 2)*Math.pow(1-t, 1)-116.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.03686 
                                        , 5.06);
        secondEquation = new BezierCurve((t) -> (189.0*Math.pow(t, 0)*Math.pow(1-t, 3)+649.5*Math.pow(t, 1)*Math.pow(1-t, 2)+627.0*Math.pow(t, 2)*Math.pow(1-t, 1)+177.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.03686
                                        , (t) -> (-116.5*Math.pow(t, 0)*Math.pow(1-t, 3)-438.0*Math.pow(t, 1)*Math.pow(1-t, 2)-526.5*Math.pow(t, 2)*Math.pow(1-t, 1)-170.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.03686
                                        , 2.94);
        thirdEquation = new BezierCurve((t) -> (177.0*Math.pow(t, 0)*Math.pow(1-t, 3)+444.0*Math.pow(t, 1)*Math.pow(1-t, 2)+466.5*Math.pow(t, 2)*Math.pow(1-t, 1)+189.5*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.03686
                                        , (t) -> (-170.0*Math.pow(t, 0)*Math.pow(1-t, 3)-501.0*Math.pow(t, 1)*Math.pow(1-t, 2)-381.0*Math.pow(t, 2)*Math.pow(1-t, 1)-117.0*Math.pow(t, 3)*Math.pow(1-t, 0)) * 0.03686
                                        , 2.87);
        
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

    public double GetDegree(Drivetrain drive, double distance) {
        // x = drive.getPose().getX() - 2.1;

        switch (state) {
            case FIRST_EQUATION:
                if (distance > firstEquation.distance) {
                    state = State.SECOND_EQUATION;
                }

                t = distance / firstEquation.distance;
                slope = -BezierDerivative.derive(firstEquation.funcX, firstEquation.funcY, t);
                return Convert.getDegrees(slope, 1);

            case SECOND_EQUATION:
                distance -= firstEquation.distance;
                if (distance > secondEquation.distance)
                    state = State.THIRD_EQUATION;
                
                t = distance / secondEquation.distance;
                slope = -BezierDerivative.derive(secondEquation.funcX, secondEquation.funcY, t);
                return Convert.getDegrees(slope, 1);

            case THIRD_EQUATION:
                distance -= firstEquation.distance + secondEquation.distance;
                if (distance > thirdEquation.distance) 
                    state = State.STOP;

                t = distance / thirdEquation.distance;
                slope = -BezierDerivative.derive(thirdEquation.funcX, thirdEquation.funcY, t);
                return Convert.getDegrees(slope, 1);
            default:
                stopMoving = true;
                return 0.0;
        }
    }
}
