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

public class Secondpath {
    

    //Creating doubles to assign functions to
    DoubleFunction<Double> firstEquationDerivative;
    DoubleFunction<Double> secondEquationDerivative;
    DoubleFunction<Double> thirdEquationDerivative;
    DoubleFunction<Double> fifthEquationDerivative;
    DoubleFunction<Double> sixthEquationDerivative;
    DecimalFormat df = new DecimalFormat("###.####");
    
    Tangent circle;

    //Currently moving
    boolean stopMoving = false;

    State state;

    public enum State {
        FIRST_EQUATION,
        SECOND_EQUATION,
        THIRD_EQUATION,
        FOURTH_EQUATION,
        FIFTH_EQUATION,
        SIXTH_EQUATION;
    }

    public void SecondInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(new Pose2d(new Translation2d(1.9, 1.152), Rotation2d.fromDegrees(0)));

        //Assigning the equations and deriving them

        DoubleFunction<Double> firstEquation = (x) -> 1 + -0.179*x + 0.4*x*x; //Desmos equations
        firstEquationDerivative = Derivative.derive(firstEquation);

        DoubleFunction<Double> secondEquation = (x) -> -0.68 + 2.05*x + -0.188*x*x;
        secondEquationDerivative = Derivative.derive(secondEquation);

        DoubleFunction<Double> thirdEquation = (x) -> 19.3 + -6.46*x + 0.987*x*x + -0.0515*(x*x*x);
        thirdEquationDerivative = Derivative.derive(thirdEquation);

        circle = new Tangent(Math.sqrt(2.8), -88.389); //Circle (radius, offset)

        DoubleFunction<Double> fifthEquation = (x) -> -30.3 + 13.6*x + -1.96*x*x + 0.0934*(x*x*x);
        fifthEquationDerivative = Derivative.derive(fifthEquation);

        DoubleFunction<Double> sixthEquation = (x) -> 4.29 + -0.121*x + -0.691*x*x + 0.186*(x*x*x) + -0.0138*(x*x*x*x);
        sixthEquationDerivative = Derivative.derive(sixthEquation);


        state = State.FIRST_EQUATION;

        stopMoving = false;
    }

    //During periodic it will drive in the direction of double degree

    public void SecondPeriodic(Drivetrain drive, Trajectory trajectory, Timer timer, RamseteController ramsete){
        double degree = GetDegree(drive, drive.distanceTravelledInMeters);
        if (!stopMoving)
            drive.drive(3.0, degree, 0);
        else{
            drive.drive(0.0, 0, 0); //Stops moving
        }
    }

    double x;
    double slope;
    public double GetDegree(Drivetrain drive, double distance){
        // 4.9 / 7.07
        x = drive.getPose().getX() - 1.9; //x-axis offset


        //Starts moving equation by equation
        switch (state) {
            case FIRST_EQUATION: //First Equation
                if (distance > 4.004) { //Length of first Equation
                    state = State.SECOND_EQUATION;
                }

                slope = -firstEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);

            case SECOND_EQUATION: //Second Equation
                distance -= 4.004;

                if (distance > 3.160) 
                    state = State.THIRD_EQUATION;
                
                slope = -secondEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1);
            

            case THIRD_EQUATION: //Third Equation
                distance -= 4.004 + 3.160;

                if (distance > 5.191) 
                    state = State.FOURTH_EQUATION;

                slope = -thirdEquationDerivative.apply(x) / 1.3;
                return Convert.getDegrees(slope, 1);

            case FOURTH_EQUATION: //Circle
                distance -= 4.004 + 3.160 + 5.191;

                if (distance > circle.circumference)
                    state = State.FIFTH_EQUATION;
                
                return -circle.getDegree(distance);

            case FIFTH_EQUATION: //Fifth equatioin
                distance -= 4.004 + 3.160 + 5.191 + circle.circumference;

                if (distance > 4.408)
                    state = State.SIXTH_EQUATION;

                slope = -fifthEquationDerivative.apply(x);
                return Convert.getDegrees(slope, 1) + 180;

            default: //Sixth Equation
                
            distance -= 4.004 + 3.160 + 5.191 + circle.circumference + 4.408;
                
            slope = -sixthEquationDerivative.apply(x);
                     
            if (x <= 0)
                stopMoving = true;
            return Convert.getDegrees(slope, 1) + 180;
        }
    }

}
