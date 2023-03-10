package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class Start3PickupRed extends DriveSegment {
    public Start3PickupRed(Swerve swerve){
        super(swerve, 
        List.of(WAYPOINT_START1.getTranslation(), RED_WAYPOINT_LEFT_OF_CS.getTranslation(),WAYPOINT_GP1.getTranslation()),
            WAYPOINT_START1.getRotation(),
            WAYPOINT_GP1.getRotation(),
            false,
            true);
   
  }
  protected static final Pose2d RED_WAYPOINT_LEFT_OF_CS = new Pose2d(3.37, 0.55, Rotation2d.fromDegrees(0));
}