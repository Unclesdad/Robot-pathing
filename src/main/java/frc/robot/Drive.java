package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive {
  Pose2d hyptheticalPose = new Pose2d();

  /** Hypothetical drive file. It doesn't do anything except... sit here and look pretty. */
  public Drive() {}

  public Command goToState(Pose2d position, double speed) {
    return new Command() {
      // this doesnt do anything
    };
  }

  public Pose2d getPose() {
    return hyptheticalPose;
  }

  public double getVelocity() {
    return 0;
  }
}
