package frc.robot.Utils;

public class Tangent {
    
    public double radius;
    private double degreeOffset;
    
    public double circumference;

    public Tangent(double radius, double degreeOffset) {
        this.radius = radius;
        this.degreeOffset = degreeOffset;

        circumference = radius * Math.PI * 2;
    }

    public double getDegree(double distance) {
        double degree = distance / circumference * 360 + degreeOffset;

        return -degree;
    }
}
