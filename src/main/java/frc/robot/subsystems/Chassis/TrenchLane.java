package frc.robot.subsystems.Chassis;

/** Trench centerline splines loaded from PathPlanner path files. */
public enum TrenchLane {
  ATLtoNTL("ATLtoNTL"),
  ATRtoNTR("ATRtoNTR");

  public final String pathName;

  TrenchLane(String pathName) {
    this.pathName = pathName;
  }
}
