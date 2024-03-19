package frc.robot.pathfinding;

public class GridBox {
    boolean obstacled = false;

    double cost = 0;

    public GridBox() { }

    public void setCost(double proposed, boolean reset) {
        cost = reset ? proposed : Math.min(cost, proposed);
    }

    /** Resets the cost of the GridBox. */
    public void resetCost() {
        cost = 0;
    }

    /** @return The cost of the GridBox. */
    public double getCost() {
        return cost;
    }

    /** Resets the obstacled state of the GridBox, meaning that it will be considered as "not an obstacle" */
    public void deobstaclize() {
        obstacled = false;
    }

    /** Turns the obstacled status of the GridBox to true, meaning that it will be considered as "an obstacle" */
    public void obstaclize() {
        obstacled = true;
    }

    /** @return Whether the GridBox is considered an obstacle or not. */
    public boolean checkObstacled() {
        return obstacled;
    }
}
