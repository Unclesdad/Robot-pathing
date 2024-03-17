package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class Drive {
    public Pose2d temp = new Pose2d();

    public Drive() {

    }

    public Pose2d pose() {
        return temp;
    }
}