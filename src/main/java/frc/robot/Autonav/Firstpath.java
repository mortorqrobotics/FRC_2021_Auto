package frc.robot.Autonav;

import java.text.DecimalFormat;
import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Drivetrain;
import frc.robot.Utils.Convert;
import frc.robot.Utils.Derivative;
import frc.robot.Utils.Tangent;

public class Firstpath {
    
    DoubleFunction<Double> firstEquationDerivative;
    DoubleFunction<Double> thirdEquationDerivative;
    DoubleFunction<Double> fourthEquationDerivative;
    DecimalFormat df = new DecimalFormat("###.####");

    boolean stopMoving = false;

    Tangent firstCircle;
    Tangent secondCircle;
    Tangent thirdCircle;

    State state;

    public enum State {
        FIRST_EQUATION,
        SECOND_EQUATION,
        THIRD_EQUATION,
        FOURTH_EQUATION,
        FIFTH_EQUATION,
        SIXTH_EQUATION,
        SEVENTH_EQUATION;
    }

    public void FirstInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(trajectory.getInitialPose());

        DoubleFunction<Double> firstEquation = (x) -> 3.87 + 0.17*x + 0.0627*(x*x) + -0.0201* (x * x * x);
        firstEquationDerivative = Derivative.derive(firstEquation);

        DoubleFunction<Double> thirdEquation = (x) -> -2.81+3.19*x+-0.511*x*x + 0.0279*(x*x*x);
        thirdEquationDerivative = Derivative.derive(thirdEquation);

        fourthEquationDerivative = (x) -> -5.9285 + 2 * 0.275 * x;

        firstCircle = new Tangent(Math.sqrt(1.2), 11.156);
        state = State.FIRST_EQUATION;

        secondCircle = new Tangent(Math.sqrt(1.2), -13.136);
        thirdCircle = new Tangent(Math.sqrt(1.2), -30.14);

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
    double slope;
    public double GetDegree(Drivetrain drive, double distance){
        // 4.9 / 7.07
        x = drive.getPose().getX() - 2.1;

        switch (state) {
            case FIRST_EQUATION:
                if (distance > 5.06) { //Distance of first equation
                    state = State.SECOND_EQUATION;
                }

                slope = -firstEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SECOND_EQUATION:
                distance -= 5.06;

                if (distance > firstCircle.circumference)
                    state = State.THIRD_EQUATION;
                
                return firstCircle.getDegree(distance);
            

            case THIRD_EQUATION:
                distance -= 5.06 + firstCircle.circumference;

                if (distance > 3.4139) 
                    state = State.FOURTH_EQUATION;

                slope = -thirdEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case FOURTH_EQUATION:
                distance -= 5.06 + firstCircle.circumference + 3.4139;

                if (distance > 5.98)
                    state = State.FIFTH_EQUATION;
                
                return -secondCircle.getDegree(distance);

            case FIFTH_EQUATION:
                distance -= 5.06 + firstCircle.circumference + 3.4139 + 5.98;

                if (distance > 4.882)
                    state = State.SIXTH_EQUATION;

                slope = -fourthEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SIXTH_EQUATION:
                distance -= 5.06 + firstCircle.circumference + 3.4139 + 5.98 + 4.882;

                if (distance > 4.018) {
                    slope = -(4.8 - drive.getPose().getY()) / (0 - drive.getPose().getX());
                    state = State.SEVENTH_EQUATION;
                }
            
                return -secondCircle.getDegree(distance);   

            default:
                if (x <= 0)
                    stopMoving = true;
                return Convert.getDegrees(slope, 1) + 180;
        }
    }

}
