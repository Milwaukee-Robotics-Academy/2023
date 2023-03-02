package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class Start3PickupBlue extends DriveSegment {
    public Start3PickupBlue(Swerve swerve){
        super(swerve, 
        List.of(WAYPOINT_START1.getTranslation(), BLUE_WAYPOINT_RIGHT_OF_CS.getTranslation(),WAYPOINT_GP1.getTranslation()),
            WAYPOINT_START1.getRotation(),
            WAYPOINT_GP1.getRotation(),
            false,
            true);
   
  }
  protected static final Pose2d BLUE_WAYPOINT_RIGHT_OF_CS = new Pose2d(3.37, 1.5, Rotation2d.fromDegrees(0));
}