package org.firstinspires.ftc.teamcode.Motion;

public class Pose2D {
    private double x;       // Forward/backward or position X
    private double y;       // Strafe/sideways or position Y
    private double heading; // Heading in radians

    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {// this is the normalized heading
        return heading;
    }

    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2D minus(Pose2D other) {
        return new Pose2D(
                this.x - other.x,
                this.y - other.y,
                this.heading - other.heading
        );
    }

    public Pose2D times(double scalar) {
        return new Pose2D(x * scalar, y * scalar, heading * scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

}

