package frc.robot.pathfinding;

public class Convenience {
  // exactly as it sounds. this just makes it easier for me

  /**
   * A record that stores two integers.
   *
   * @param intOne It's an integer.
   * @param intTwo You won't believe this. It's another integer. Groundbreaking stuff huh
   */
  public record Point(int intOne, int intTwo) {
    public final int getX() {
      return intOne;
    }

    public final int getY() {
      return intTwo;
    }
  }
}
