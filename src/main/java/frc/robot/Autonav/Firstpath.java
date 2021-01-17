package frc.robot.Autonav;

import java.text.DecimalFormat;
import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Drivetrain;
import frc.robot.Utils.ConvertToDegrees;
import frc.robot.Utils.Derivative;

public class Firstpath {
    
    DoubleFunction<Double> firstEquationDerivative;
    DecimalFormat df = new DecimalFormat("###.####");

    public void FirstInit(Drivetrain drive, Trajectory trajectory){

        drive.resetOdometry(trajectory.getInitialPose());

        DoubleFunction<Double> firstEquation = (x) -> 3.87 + 0.17*x + 0.0627*(x*x) + -0.0201* (x * x * x);
        firstEquationDerivative = Derivative.derive(firstEquation);
    }

    public void FirstPeriodic(Drivetrain drive, Trajectory trajectory, Timer timer, RamseteController ramsete){

        drive.drive(3.0, GetDegree(drive.distanceTravelledInMeters), 0);

    }

    public double GetDegree(double distance){
        if (distance <= 10){
            double slope = -firstEquationDerivative.apply(distance);
            return ConvertToDegrees.getDegrees(slope, 1);
        } else if (distance <= 20){
            return StepTwo();
        } else if (distance <= 30){
            return StepThree();
        } else if (distance > 30 && distance <= 40){
            return StepFour();
        } else if (distance > 40 && distance <= 50){
            return StepFive();
        } else if (distance > 50 && distance <= 60){
            return StepSix();
        } else {
            return StepSeven();
        }
    }

    public double StepOne (){
        double degree = 0;
        return degree;
    }

    public double StepTwo (){
        double degree = 90;
        return degree;
    }

    public double StepThree (){
        double degree = 180;
        return degree;
    }

    public double StepFour (){
        double degree = 270;
        return degree;
    }

    public double StepFive (){
        double degree = 0;
        return degree;
    }

    public double StepSix (){
        double degree = 90;
        return degree;
    }

    public double StepSeven (){
        double degree = 180;
        return degree;
    }

}
