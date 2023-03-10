package frc.robot.autos;

import java.util.List;

import frc.robot.subsystems.Swerve;

public class Start2CrossAndCS extends DriveSegment {
    public Start2CrossAndCS(Swerve swerve){
        super(swerve, 
        List.of(WAYPOINT_START2.getTranslation(), WAYPOINT_CENTER_CROSS.getTranslation()),
            WAYPOINT_START1.getRotation(),
            WAYPOINT_CHARGESTATION.getRotation(),
            false,
            true);
      }
}