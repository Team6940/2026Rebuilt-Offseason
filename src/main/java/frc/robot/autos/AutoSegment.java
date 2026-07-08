package frc.robot.autos;

import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.List;

/** Static metadata for each selectable autonomous segment. */
public enum AutoSegment {
  UNUSED("UNUSED", false, IntakeMode.OFF, List.of()),
  LST_LINT1("LSt-LInt1", true, IntakeMode.INTAKE, List.of("LInt1-LSh1")),
  LINT1_LSH1("LInt1-LSh1", true, IntakeMode.MID, List.of("LSh1-LInt2", "LSh1-LInt2OverMid")),
  LSH1_LINT2("LSh1-LInt2", true, IntakeMode.INTAKE, List.of("LInt2-LSh2")),
  LINT2_LSH2("LInt2-LSh2", true, IntakeMode.MID, List.of("LSh2-LEnd", "LSh2-LEndInt3")),
  LSH2_LEND("LSh2-LEnd", true, IntakeMode.INTAKE, List.of()),
  LSH2_LENDINT3("LSh2-LEndInt3", true, IntakeMode.INTAKE, List.of()),
  LST2_LDEPOT("LSt2-LDepot", true, IntakeMode.INTAKE, List.of("LDepot-LSh3")),
  LDEPOT_LSH3("LDepot-LSh3", true, IntakeMode.INTAKE, List.of("LSh3-LInt2b")),
  LSH3_LINT2B("LSh3-LInt2b", true, IntakeMode.INTAKE, List.of("LInt2-LSh2")),
  RST_RINT1("RSt-RInt1", true, IntakeMode.INTAKE, List.of("RInt1-RSh1")),
  RINT1_RSH1("RInt1-RSh1", true, IntakeMode.MID, List.of("RSh1-RInt2", "RSh1-RInt2OverMid")),
  RSH1_RINT2("RSh1-RInt2", true, IntakeMode.INTAKE, List.of("RInt2-RSh2")),
  RINT2_RSH2("RInt2-RSh2", true, IntakeMode.MID, List.of("RSh2-REnd", "RSh2-REndInt3")),
  RSH2_REND("RSh2-REnd", true, IntakeMode.INTAKE, List.of()),
  RSH2_RENDINT3("RSh2-REndInt3", true, IntakeMode.INTAKE, List.of()),
  RST_RINT1_OVER_MID("RSt-RInt1OverMid", true, IntakeMode.INTAKE, List.of("RInt1OverMid-RSh1", "RInt1OverMid-ROppHub")),
  RINT1_OVER_MID_RSH1("RInt1OverMid-RSh1", true, IntakeMode.MID, List.of("RSh1-RInt2OverMid")),
  RSH1_RINT2_OVER_MID("RSh1-RInt2OverMid", true, IntakeMode.INTAKE, List.of("RInt2OverMid-RSh2")),
  RINT2_OVER_MID_RSH2("RInt2OverMid-RSh2", true, IntakeMode.MID, List.of("RSh2-REnd", "RSh2-REndInt3")),
  ROPPHUB_RSH2("ROppHub-RSh2", true, IntakeMode.MID, List.of("RSh2-REnd", "RSh2-REndInt3")),
  SHOOT_SCORE("Shoot Score", false, IntakeMode.OFF, List.of()),
  SHOOT_PASS("Shoot Pass", false, IntakeMode.OFF, List.of());

  private final String displayName;
  private final boolean pathSegment;
  private final IntakeMode intakeMode;
  private final List<String> nextPaths;

  AutoSegment(String displayName, boolean pathSegment, IntakeMode intakeMode, List<String> nextPaths) {
    this.displayName = displayName;
    this.pathSegment = pathSegment;
    this.intakeMode = intakeMode;
    this.nextPaths = nextPaths;
  }

  public String getDisplayName() {
    return displayName;
  }

  public boolean isPathSegment() {
    return pathSegment;
  }

  public IntakeMode getIntakeMode() {
    return intakeMode;
  }

  public boolean isShootSegment() {
    return this == SHOOT_SCORE || this == SHOOT_PASS;
  }

  public String getPathName() {
    return displayName;
  }

  public boolean canShootAfter() {
    return pathSegment;
  }

  public List<AutoSegment> getNextPaths() {
    return nextPaths.stream().map(AutoSegment::fromPathName).toList();
  }

  public static List<AutoSegment> getStartingSegments() {
    return List.of(
        LST_LINT1,
        LST2_LDEPOT,
        RST_RINT1,
        RST_RINT1_OVER_MID);
  }

  public static AutoSegment fromPathName(String pathName) {
    for (AutoSegment segment : values()) {
      if (segment.displayName.equals(pathName)) {
        return segment;
      }
    }
    return UNUSED;
  }
}
