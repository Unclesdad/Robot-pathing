package frc.robot.pathfinding;

public class PathfindingConstants {
  /*
   * The distance from the start point of the path, before it begins to
   * start focusing on the target point.
   * Decreasing this number increases performance but decreases reliability,
   * and increasing this number decreases performance but increases reliability.
   * Each increase by 1, the radius at which it starts to focus increases by
   * GriddedField.GRID_SIDE_LENGTH cm.
   */
  public static final int COST_CREATIVITY = 60;

  /*
   * The amount of times a GridBox will allow itself to be assigned a new cost
   * before it stops taking new assignments.
   *
   * This number should be between 1 and 8.
   * The higher the number, the more precise the algorithm will be, but increasing
   * the number grants diminishing returns on precision while exponentially decreasing
   * performance. It is highly advised to keep this number low, around 2-4.
   */
  public static final int MAX_ASSIGNED_VALUE = 3;

  /*
   * The side length of each individual grid used in the GriddedField.
   */
  public static final int GRID_SIDE_LENGTH = 4;
}
