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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Thirdpath {
    
    

    DoubleFunction<Double> firstEquationDerivative;
    DoubleFunction<Double> secondEquationDerivative;
    DoubleFunction<Double> thirdEquationDerivative;
    DoubleFunction<Double> fourthEquationDerivative;
    DoubleFunction<Double> fifthEquationDerivative;
    DoubleFunction<Double> sixthEquationDerivative;
    DoubleFunction<Double> seventhEquationDerivative;
    DoubleFunction<Double> eighthEquationDerivative;
    DecimalFormat df = new DecimalFormat("###.####");

    boolean stopMoving = false;

    Tangent circle;

    State state;

    public enum State {
        FIRST_EQUATION,
        SECOND_EQUATION,
        THIRD_EQUATION,
        FOURTH_EQUATION,
        FIFTH_EQUATION,
        SIXTH_EQUATION,
        SEVENTH_EQUATION,
        EIGHTH_EQUATION;
    }

    public void ThirdInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(new Pose2d(new Translation2d(1.84, 4.17), Rotation2d.fromDegrees(0)));

        DoubleFunction<Double> firstEquation = (x) -> 3.99 + 0.41*x + -1.04*x*x + 0.782*x*x*x; //formula 1
        firstEquationDerivative = Derivative.derive(firstEquation);

        DoubleFunction<Double> secondEquation = (x) -> 19.6 + -10.6*x + 2.06*x*x + -0.133*x*x*x; //formula 2
        secondEquationDerivative = Derivative.derive(secondEquation);

        DoubleFunction<Double> thirdEquation = (x) -> 90.29 + -34.25*(x+0.7) + 3.275*(x+0.7)*(x+0.7); //formula 3
        thirdEquationDerivative = Derivative.derive(thirdEquation);

        DoubleFunction<Double> fourthEquation = (x) -> -2.2982*x*x*x + 49.381*x*x - 353.46*x + 844.48; //formula 4
        fourthEquationDerivative = Derivative.derive(fourthEquation);

        DoubleFunction<Double> fifthEquation = (x) -> 10.8 - 2.35*x + 0.15*x*x; //formula 5
        fifthEquationDerivative = Derivative.derive(fifthEquation);

        DoubleFunction<Double> sixthEquation = (x) -> 1082 + -237*x + 13*x*x; //formula 6
        sixthEquationDerivative = Derivative.derive(sixthEquation);

        DoubleFunction<Double> seventhEquation = (x) -> 88.7 + -14.49*x + 0.6219*x*x; //formula 7
        seventhEquationDerivative = Derivative.derive(seventhEquation);

        eighthEquationDerivative = (x) -> x * 0;

        state = State.FIRST_EQUATION;

        stopMoving = false;
    }

    public void ThirdPeriodic(Drivetrain drive, Trajectory trajectory, Timer timer, RamseteController ramsete){
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
        x = drive.getPose().getX() - 2;

        switch (state) {
            case FIRST_EQUATION:
                if (distance > 3.482) {
                    state = State.SECOND_EQUATION;
                }

                slope = -firstEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SECOND_EQUATION:
            
                distance -= 3.482;

                if (distance > 5.973)
                    state = State.THIRD_EQUATION;
                
                slope = -secondEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);
            

            case THIRD_EQUATION:

                distance -= 3.482 + 5.973;

                if (distance > 5.154) 
                    state = State.FOURTH_EQUATION;

                slope = -thirdEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case FOURTH_EQUATION:
                distance -= 3.482 + 5.973 + 5.154;

                if (distance > 5.758)
                    state = State.FIFTH_EQUATION;
                
                    slope = -fourthEquationDerivative.apply(x);
                    return Convert.getDegrees(slope, 1);

            case FIFTH_EQUATION:
                // stopMoving = true;
                distance -= 3.482 + 5.973 + 5.154 + 5.758;

                if (distance > 1.641)
                    state = State.SIXTH_EQUATION;

                slope = -fifthEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SIXTH_EQUATION:
                distance -= 3.482 + 5.973 + 5.154 + 5.758 + 1.641;

                if (distance > 7)
                    state = State.SEVENTH_EQUATION;

                slope = -sixthEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SEVENTH_EQUATION:
            // stopMoving = true;
                distance -= 3.482 + 5.973 + 5.154 + 5.758 + 1.641 + 7;

                if (distance > 2.912)
                    state = State.EIGHTH_EQUATION;

                slope = -seventhEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            default:
                
                distance -= 3.482 + 5.973 + 5.154 + 5.758 + 1.641 + 7 + 2.912;
                    
                if (drive.getPose().getX() > 14.7) {
                    stopMoving = true;
                }

                slope = 0;
                return Convert.getDegrees(slope, 1); 
        }
    }

}
