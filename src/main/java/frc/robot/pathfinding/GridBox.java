package frc.robot.pathfinding;

public class GridBox {
    boolean obstacled = false;

    double cost = 0;

    public GridBox() { }

    public void setCost(double proposed, boolean reset) {
        cost = reset ? proposed : Math.min(cost, proposed);
    }

    public void resetCost() {
        cost = 0;
    }

    public double getCost() {
        return cost;
    }

    public void resetObstacled() {
        obstacled = false;
    }

    public void obstaclize() {
        obstacled = true;
    }

    public boolean checkObstacled() {
        return obstacled;
    }
}
