package org.firstinspires.ftc.teamcode.Motion;
import com.arcrobotics.ftclib.controller.PIDController;

public class TrajectoryFollower {

    public static class PIDCoefficients {
        public final double kP, kI, kD;

        public PIDCoefficients(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
    public static class TrapezoidalMotionProfile {
        private final double distance, maxVel, maxAccel;

        public TrapezoidalMotionProfile(double distance, double maxVel, double maxAccel) {
            this.distance = distance;
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
        }
        public double getVelocityAtDistance(double d) {
            d = Math.max(0, Math.min(distance, d)); // Clamp to [0, total distance]

            double accelDist = (maxVel * maxVel) / (2 * maxAccel);

            if (distance < 2 * accelDist) {
                // Triangular profile
                double halfDist = 0.5 * distance;
                if (d < halfDist) {
                    return Math.sqrt(2 * maxAccel * d);
                } else {
                    return Math.sqrt(2 * maxAccel * (distance - d));
                }
            } else {
                if (d < accelDist) {
                    return Math.sqrt(2 * maxAccel * d);
                } else if (d < (distance - accelDist)) {
                    return maxVel;
                } else {
                    return Math.sqrt(2 * maxAccel * (distance - d));
                }
            }
        }

        public double getAccelerationAtDistance(double d) {
            d = Math.max(0, Math.min(distance, d)); // Clamp to [0, total distance]

            double accelDist = (maxVel * maxVel) / (2 * maxAccel);

            if (distance < 2 * accelDist) {
                // Triangular profile
                double halfDist = 0.5 * distance;
                if (d < halfDist) {
                    return maxAccel;
                } else {
                    return -maxAccel;
                }
            } else {
                if (d < accelDist) {
                    return maxAccel;
                } else if (d < (distance - accelDist)) {
                    return 0;
                } else {
                    return -maxAccel;
                }
            }
        }
    }

    private PIDController xPID;
    private PIDController yPID;
    private PIDController headingPID;

    private double kV, kA, kStatic; // Feedforward gains

    public TrajectoryFollower(
            PIDCoefficients xCoeffs,
            PIDCoefficients yCoeffs,
            PIDCoefficients headingCoeffs,
            double kV, double kA, double kStatic) {

        this.xPID = new PIDController(xCoeffs.kP, xCoeffs.kI, xCoeffs.kD);
        this.yPID = new PIDController(yCoeffs.kP, yCoeffs.kI, yCoeffs.kD);
        this.headingPID = new PIDController(headingCoeffs.kP, headingCoeffs.kI, headingCoeffs.kD);

        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }

    public Pose2D update(Pose2D currentPose,
                         Pose2D targetPose,
                         Pose2D targetVelocity,
                         Pose2D targetAcceleration) {

        Pose2D poseError = targetPose.minus(currentPose);

        // Feedback terms (PID)
        xPID.setSetPoint(0);
        yPID.setSetPoint(0);
        headingPID.setSetPoint(0);

        double feedbackX = xPID.calculate(poseError.getX());
        double feedbackY = yPID.calculate(poseError.getY());
        double feedbackHeading = headingPID.calculate(poseError.getHeading());

        // Feedforward terms
        double ffX = kV * targetVelocity.getX() + kA * targetAcceleration.getX();
        double ffY = kV * targetVelocity.getY() + kA * targetAcceleration.getY();
        double ffHeading = kV * targetVelocity.getHeading() + kA * targetAcceleration.getHeading();

        // Sum of feedforward + feedback
        double vx = ffX + feedbackX;
        double vy = ffY + feedbackY;
        double omega = ffHeading + feedbackHeading;

        // Apply kStatic if moving
        if (Math.abs(vx) > 1e-4) vx += Math.signum(vx) * kStatic;
        if (Math.abs(vy) > 1e-4) vy += Math.signum(vy) * kStatic;
        if (Math.abs(omega) > 1e-4) omega += Math.signum(omega) * kStatic;

        return new Pose2D(vx, vy, omega);
    }
    public static Pose2D projectScalarMotion(Pose2D currentPose,
                                             Pose2D targetPose,
                                             double scalarVelocity,
                                             double scalarAcceleration,
                                             double angularVelocity,
                                             double angularAcceleration) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);

        double unitX = (distance > 1e-4) ? dx / distance : 0;
        double unitY = (distance > 1e-4) ? dy / distance : 0;

        return new Pose2D(
                scalarVelocity * unitX,
                scalarVelocity * unitY,
                angularVelocity
        );
    }

    public static Pose2D projectScalarAcceleration(Pose2D currentPose,
                                                   Pose2D targetPose,
                                                   double scalarAcceleration,
                                                   double angularAcceleration) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);

        double unitX = (distance > 1e-4) ? dx / distance : 0;
        double unitY = (distance > 1e-4) ? dy / distance : 0;

        return new Pose2D(
                scalarAcceleration * unitX,
                scalarAcceleration * unitY,
                angularAcceleration
        );
    }

    public void reset() {
        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }
}


