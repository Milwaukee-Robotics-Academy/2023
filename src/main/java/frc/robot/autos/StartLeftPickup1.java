package frc.robot.autos;

import java.util.List;

import frc.robot.subsystems.Swerve;

public class StartLeftPickup1 extends DriveSegment {
    public StartLeftPickup1(Swerve swerve){
        super(swerve, 
        List.of(WAYPOINT_START1.getTranslation(), WAYPOINT_RIGHT_OF_CS.getTranslation(),WAYPOINT_GP1.getTranslation()),
            WAYPOINT_START1.getRotation(),
            WAYPOINT_GP1.getRotation(),
            false,
            true);
      }
}