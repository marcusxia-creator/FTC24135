package org.firstinspires.ftc.teamcode.Motion;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class IMUReader implements Runnable {
    private final double[] buffer;
    public RobotHardware robot;                             // Bring in robot hardware configuration
    private int index = 0;
    private int count = 0;
    private int bufferSize;

    private volatile boolean running = true;

    public IMUReader(RobotHardware robot, int bufferSize) {
        this.robot = robot;
        this.bufferSize = bufferSize;
        this.buffer = new double[bufferSize];
    }

    public void start() {
        Thread thread = new Thread(this);
        thread.start();
    }

    public void stop() {
        running = false;
    }

    @Override
    public void run() {
        while (running) {
            double heading = robot.imu.getRobotYawPitchRollAngles().getYaw();

            synchronized (this) {
                buffer[index] = heading;
                index = (index + 1) % bufferSize;
                if (count < bufferSize) count++;
            }

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public synchronized double getAverageHeading() {
        double sumSin = 0;
        double sumCos = 0;

        for (int i = 0; i < count; i++) {
            double radians = Math.toRadians(buffer[i]);
            sumSin += Math.sin(radians);
            sumCos += Math.cos(radians);
        }

        if (count == 0) return 0;

        double avgRadians = Math.atan2(sumSin / count, sumCos / count);
        double avgDegrees = Math.toDegrees(avgRadians);

        if (avgDegrees < 0) avgDegrees += 360;
        return avgDegrees;
    }
}

