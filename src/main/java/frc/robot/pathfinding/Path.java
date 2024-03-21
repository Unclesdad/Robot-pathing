package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drive;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class Path {
  public final Pose2d endingPoint;
  // How many periods before the robot updates the field and calculates a new path.
  // Increasing this number improves performance but also lengthens reaction time.
  private final int REFRESH_TIME = 5;
  private int refreshMeter = 5;

  public final List<Obstacle> obstacles;

  private static final AffineTransform nullTransform = new AffineTransform();

  private GriddedField field;

  private CharliesAstar aStar;

  private Drive drive;

  public Path(
      Pose2d startingPoint,
      Pose2d endingPoint,
      List<Obstacle> stationaryObstacles,
      GriddedField field,
      Drive drive) {
    this.endingPoint = endingPoint;
    this.obstacles = stationaryObstacles;
    this.field = field;
    this.drive = drive;
    aStar =
        new CharliesAstar(
            field, field.coordsToBox(new Translation2d(endingPoint.getX(), endingPoint.getY())));
  }

  /**
   * Static method to find out whether a point can be driven straight to from another point.
   *
   * @param startPoint The starting point.
   * @param setPoint The ending point.
   * @return whether the ending point can be driven straight to from the starting point.
   */
  public static boolean pointSeeable(
      Pose2d startPoint, Pose2d setPoint, List<Obstacle> obstaclelist) {
    AtomicBoolean seeable = new AtomicBoolean(true);
    Line2D beeline =
        new Line2D.Double(startPoint.getX(), startPoint.getY(), setPoint.getX(), setPoint.getY());
    obstaclelist.forEach(
        obstacle ->
            seeable.set(
                seeable.get() && shapeIntersectsLine(obstacle.obstacleProjection(), beeline)));
    return seeable.get();
  }

  public static boolean shapeIntersectsLine(java.awt.Shape shape, Line2D line) {
    PathIterator iterator = shape.getPathIterator(nullTransform);
    double[] currCoords = new double[6];
    double[] prevCoords = new double[2];

    iterator.currentSegment(prevCoords);

    while (!iterator.isDone()) {
      int segmentType = iterator.currentSegment(currCoords);

      if (segmentType != PathIterator.SEG_MOVETO) {
        Line2D.Double segment =
            new Line2D.Double(prevCoords[0], prevCoords[1], currCoords[0], currCoords[1]);

        if (segment.intersectsLine(line)) {
          return true;
        }
      }

      prevCoords[0] = currCoords[0];
      prevCoords[0] = currCoords[1];

      iterator.next();
    }
    return false;
  }

  /**
   * Nonstatic method to find out whether the goal of the robot can be driven straight to from where
   * the robot is.
   *
   * @return whether the goal can be driven straight to.
   */
  public boolean setPointSeeable(Pose2d startingPoint) {
    return pointSeeable(startingPoint, endingPoint, obstacles);
  }

  /**
   * Calculates a path from the starting point to the ending point which avoids all obstacles.
   *
   * @param startingPoint The current position of the robot as a Translation2d.
   * @param endingPoint The goal position of the robot as a Translation2d.
   * @param movingObstacles The temporary obstacles of which will be placed on the field for the
   *     calculation.
   * @return A list of Translation2ds which represent the coordinate path of the robot.
   */
  public List<Translation2d> calculatePath(
      Translation2d startingPoint, List<Obstacle> movingObstacles) {
    field.addTempObstacles(movingObstacles);
    return aStar.pathMaker(startingPoint, endingPoint.getTranslation());
  }

  /**
   * Sends the robot to drive to the next point on its path toward the goal.
   *
   * @param movingObstacles The new position of the obstacles.
   * @param speed The speed wanted at the next pose.
   * @return A command to send the robot to the next position in its heroic journey.
   */
  public Command goToNextPose(List<Obstacle> movingObstacles, double speed) {
    field.addTempObstacles(movingObstacles);
    if (refreshMeter >= REFRESH_TIME) {
      aStar.assignCosts(field.coordsToBox(endingPoint.getTranslation()));
      refreshMeter = 0;
    } else {
      refreshMeter++;
    }
    return drive.goToState(
        new Pose2d(aStar.nextPos(drive.getPose().getTranslation()), endingPoint.getRotation()),
        speed);
  }
}
