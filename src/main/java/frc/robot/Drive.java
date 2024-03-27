package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathfinding.GriddedField;
import frc.robot.pathfinding.Obstacle;
import frc.robot.pathfinding.Path;
import java.util.List;

public class Drive {
  Pose2d hyptheticalPose = new Pose2d();

  Path path;
  GriddedField field;

  public PIDController pid =
      new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  /**
   * Hypothetical drive file. It doesn't do anything except sit here, look pretty, and provide
   * placeholder methods.
   */
  public Drive() {
    field = new GriddedField();
  }

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

  public Command pathFind(List<Obstacle> obstacles) {
    
  }
}
