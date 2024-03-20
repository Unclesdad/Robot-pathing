package frc.robot.pathfinding;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.pathfinding.Convenience.Point;

public class CharliesAstar {
  
  /* The distance from the start point of the path, before it begins to 
  * start focusing on the target point. 
  * Decreasing this number increases performance but decreases reliability,
  * and increasing this number decreases performance but increases reliability.
  * Each increase by 1, the radius at which it starts to focus increases by GriddedField.GRID_SIDE_LENGTH cm.
  */
  private final int COST_CREATIVITY = 60;

  private GriddedField field;

  private GridBox setPoint;

  // The set of translations to adjacent squares. Starts at the right, rotates counter-clockwise.
  public static final Point[] intpair = {
    new Point(1, 0),
    new Point(1, 1),
    new Point(0, 1),
    new Point(-1, 1),
    new Point(-1, 0),
    new Point(-1, -1),
    new Point(0, -1),
    new Point(1, -1)
  };

  // The set of translations which move diagonally.
  public static final List<Point> diagBoxes = List.of(
    new Point(1, 1), new Point(-1, 1), new Point(-1, -1), new Point(1, -1)
  );

  public CharliesAstar(GriddedField field, GridBox setPoint) {
    this.field = field;
    this.setPoint = setPoint;
  }

  /**
   * Finds the angle, as a Rotation2d, from one GridBox to another.
   * @param setPoint The ending point of the vector that the angle will be based off of.
   * @param currentBox The starting point of the vector that the angle will be based off of.
   * @return A Rotation2d with the angle from the vector created.
   */
  public static Rotation2d pointDirection(GridBox setPoint, GridBox currentBox) {
    return new Translation2d(setPoint.getX() - currentBox.getX(), setPoint.getY() - currentBox.getY()).getAngle();
  }

  /**
   * Shortcut to whether the movement is diagonal. Will only work if given items from CharliesAstar.intpair.
   * This method exists for optimization reasons. The long way around this problem is to calculate the angle
   * and then determine whether it is diagonal from there. 
   * @param p A Point from intpair.
   * @return If the point is in CharliesAstar.diagBoxes.
   */
  private static boolean diagonalMovement(Point p) {
    return diagBoxes.contains(p);
  }

  private static double findCost(Point p) {
    return diagonalMovement(p) ? Constants.rootwo : 1;
  }

  /**
   * Returns three pairs of integers, with them representing the translation needed to get the three
   * boxes in the general direction of the rotation.
   * @param rotation The rotation inputted to get the three boxes
   * @return Three pairs of integers which represent (x,y) translation to the three boxes
   */
  public static Point[] threeRelevantCoords(Rotation2d rotation, boolean reverse) {
    /* Using degrees because it's more memory efficient than an
     * irrational decimal (multiple of pi) */
    double degrees = rotation.getDegrees();

    /* Converting to an int rounds down. This gives us the general rotation
     * (right = 0, top right = 1, top left = 3, etc) */
    int directionChange = ((int) (degrees + 22.5) / 45);

    // Assigning the three values to result. Sadly, ternary operators don't work with arrays.
    Point[] result;
    if (!reverse) {
      result =
          new Point[] {
            intpair[directionChange % 8],
            intpair[(directionChange - 1) % 8],
            intpair[(directionChange + 1) % 8]
          };
    } else {
      result =
          new Point[] {
            intpair[(directionChange + 4) % 8],
            intpair[(directionChange + 3) % 8],
            intpair[(directionChange + 5) % 8]
          };
    }

    return result;
  }

  /**
   * Returns five pairs of integers, with them representing the translation needed to get the five
   * boxes in the general direction of the rotation.
   *
   * @param rotation The rotation inputted to get the five boxes.
   * @return Five pairs of integers which represent (x,y) translation to the five boxes.
   */
  public static Point[] fiveRelevantCoords(Rotation2d rotation, boolean reverse) {
    /* Using degrees because it's more memory efficient than an
     * irrational decimal (multiple of pi) */
    double degrees = rotation.getDegrees();

    /* Converting to an int rounds down. This gives us the general rotation
     * (right = 0, top right = 1, top left = 3, etc) */
    int directionChange = ((int) (degrees + 22.5) / 45);

    // Assigning the three values to result. Sadly, ternary operators don't work with arrays.
    Point[] result;
    if (!reverse) {
      result =
          new Point[] {
            intpair[directionChange % 8],
            intpair[(directionChange - 1) % 8],
            intpair[(directionChange + 1) % 8],
            intpair[(directionChange + 2) % 8],
            intpair[(directionChange - 2) % 8]
          };
    } else {
      result =
          new Point[] {
            intpair[(directionChange + 4) % 8],
            intpair[(directionChange + 3) % 8],
            intpair[(directionChange + 5) % 8],
            intpair[(directionChange + 2) % 8],
            intpair[(directionChange + 6) % 8]
          };
          // Not very beautiful code, is it? Unfortunately, it is the most effective.
    }
    return result;
  }

  /**
   * Assigns the starting point to one grid box, initiating a chain reaction and a positive feedback
   * loop to assign nearly all GridBoxes a number relating to their supposed pathway.
   * @param startingPoint The goal of the robot. It is labeled "Starting point," however this is 
   * the starting point of the ALGORITHM, NOT the ROBOT. 
   */
  public void firstCostAssign(GridBox startingPoint) {
    startingPoint.setCost(startingPoint, 0, true);
    assignCosts(startingPoint);
  }

  /**
   * The chain reaction for the cost assignment. This utilizes recursion 
   * to make sure that no important box is left unassigned.
   * After every box is assigned, it assigns a certain amount of boxes around them.
   * If the boxes are past a certain distance from the starting point, they will begin
   * to only assign costs to boxes in front of them in the direction of the set point,
   * so that unimportant boxes in the corner are not assigned, as a method of optimization.
   * @param box The box which is currently being assigned.
   */
  public void assignCosts(GridBox box) {
    Rotation2d rotation = pointDirection(setPoint, box);

    if (true /* not goal point */) {
      if (box.getCost() < COST_CREATIVITY) {
        for (Point i : intpair) {
          GridBox proposedBox = field.field()[box.getX() + i.getX()][box.getY() + i.getY()];
          box.setCost(proposedBox, proposedBox.getCost() + findCost(i), false);
          if (proposedBox.assignable(box)) {
            assignCosts(proposedBox);
          }
        }
      } else {
        for (Point i : fiveRelevantCoords(rotation, true)) {
          GridBox proposedBox = field.field()[box.getX() + i.getX()][box.getY() + i.getY()];
          box.setCost(proposedBox, proposedBox.getCost() + findCost(i), false);
        }
        for (Point i : threeRelevantCoords(rotation, false)) {
          GridBox proposedBox = field.field()[box.getX() + i.getX()][box.getY() + i.getY()];
          if (proposedBox.assignable(box)) {
            assignCosts(proposedBox);
          }
        }
      }
    }
  }
}
