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
        SIXTH_EQUATION;
    }

    public void ThirdInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(new Pose2d(new Translation2d(1.9, 1.152), Rotation2d.fromDegrees(0)));

        DoubleFunction<Double> firstEquation = (x) -> x; //formula 1
        firstEquationDerivative = Derivative.derive(firstEquation);

        DoubleFunction<Double> secondEquation = (x) -> x; //formula 2
        secondEquationDerivative = Derivative.derive(secondEquation);

        DoubleFunction<Double> thirdEquation = (x) -> x; //formula 3
        thirdEquationDerivative = Derivative.derive(thirdEquation);

        DoubleFunction<Double> fourthEquation = (x) -> x; //formula 4
        fourthEquationDerivative = Derivative.derive(fourthEquation);

        DoubleFunction<Double> fifthEquation = (x) -> x; //formula 5
        fifthEquationDerivative = Derivative.derive(fifthEquation);

        DoubleFunction<Double> sixthEquation = (x) -> x; //formula 6
        sixthEquationDerivative = Derivative.derive(sixthEquation);


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
        // 4.9 / 7.07
        x = drive.getPose().getX() - 1.9;

        switch (state) {
            case FIRST_EQUATION:
                if (distance > 0) {
                    state = State.SECOND_EQUATION;
                }

                slope = -firstEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SECOND_EQUATION:
                distance -= 0;

                if (distance > 00)
                    state = State.THIRD_EQUATION;
                
                slope = secondEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);
            

            case THIRD_EQUATION:
                distance -= 0 + 00;

                if (distance > 000) 
                    state = State.FOURTH_EQUATION;

                slope = -thirdEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case FOURTH_EQUATION:
                distance -= 0 + 00 + 000;

                if (distance > 0000)
                    state = State.FIFTH_EQUATION;
                
                    slope = -fourthEquationDerivative.apply(x);
                    return Convert.getDegrees(slope, 1);

            case FIFTH_EQUATION:
                distance -= 0 + 00 + 000 + 0000;

                if (distance > 00000)
                    state = State.SIXTH_EQUATION;

                slope = -fifthEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            default:
                
            distance -= 0 + 00 + 000 + 0000 + 00000;
                
            slope = -sixthEquationDerivative.apply(x);
            return Convert.getDegrees(slope, 1); 
        }
    }

}
