package frc.robot.pathfinding;

import java.util.List;

public class GridBox {
  private boolean obstacled = false;

  private int assignedValue = 0;

  public static final double MAX_ASSIGNED_VALUE = 3;

  private double cost = 0;

  private int XValue;
  private int YValue;

  private List<GridBox> ignoreList = List.of();

  public GridBox(int XValue, int YValue) {
    this.XValue = XValue;
    this.YValue = YValue;
  }

  /**
   * Sets the cost of the GridBox.
   *
   * @param p the point from which the proposed cost is being taken from.
   * @param proposed The proposed number to be set as the cost.
   * @param reset A boolean. If true, the cost will be set to proposed. If false, the cost will be
   *     set to the minimum of the current cost and the proposed cost.
   */
  public void setCost(GridBox p, double proposed, boolean reset) {
    ignoreList.add(p);
    cost = reset ? proposed : Math.min(cost, proposed);
    assignedValue++;
  }

  /** Resets the cost, assigned value and ignore list of the GridBox. */
  public void reset() {
    cost = 0;
    assignedValue = 0;
    ignoreList.clear();
  }

  /**
   * @return The cost of the GridBox.
   */
  public double getCost() {
    return cost;
  }

  /**
   * Resets the obstacled state of the GridBox, meaning that it will be considered as "not an
   * obstacle"
   */
  public void deobstaclize() {
    obstacled = false;
  }

  /**
   * Turns the obstacled status of the GridBox to true, meaning that it will be considered as "an
   * obstacle"
   */
  public void obstaclize() {
    obstacled = true;
  }

  /**
   * @return Whether the GridBox is considered an obstacle or not.
   */
  public boolean checkObstacled() {
    return obstacled;
  }

  /**
   * @return The X value of the GridBox.
   */
  public int getX() {
    return XValue;
  }

  /**
   * @return The Y value of the GridBox.
   */
  public int getY() {
    return YValue;
  }

  /**
   * Finds whether or not the grid should have its neighbor's cost considered. 
   * This method mainly exists for performance reasons.
   * @param p The gridbox from which the proposed new cost is coming from. 
   * @return Whether or not the grid will take into accound its neighbor's cost.
   */
  public boolean assignable(GridBox p) {
    return assignedValue <= MAX_ASSIGNED_VALUE && !(ignoreList.contains(p)) && !obstacled;
  }
}
