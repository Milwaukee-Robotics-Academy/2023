package frc.robot.autos;

import java.util.List;
import frc.robot.subsystems.Swerve;

public class Start2Balance  extends DriveSegment {
        public Start2Balance(Swerve swerve){
            super(swerve, 
            List.of(WAYPOINT_START2.getTranslation(), WAYPOINT_CHARGESTATION.getTranslation()),
                WAYPOINT_START1.getRotation(),
                WAYPOINT_CHARGESTATION.getRotation(),
                true,
                true);
          }
    }    
