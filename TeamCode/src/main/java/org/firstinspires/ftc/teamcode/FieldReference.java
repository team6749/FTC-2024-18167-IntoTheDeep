package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FieldReference {

    public final int tag_id;
    public float rotation ;
    public final Vector2d pos, axis_i, axis_j;

    public FieldReference(int id, float posX, float posY, float angle ) {
        this.tag_id = id;
        double rotrad = Math.toRadians(angle) ;
        double rotsin = Math.sin( rotrad ) ;
        double rotcos = Math.cos( rotrad ) ;
        this.pos = new Vector2d(posX, posY);
        this.rotation = angle ;
        this.axis_i = new Vector2d( rotcos, rotsin ) ;
        this.axis_j = new Vector2d( - rotsin, rotcos ) ;
    }

    Pose2d locate(AprilTagDetection pic )
    {
        double sighting = Math.toRadians(pic.ftcPose.bearing - pic.ftcPose.yaw - this.rotation);
        double c = Math.cos(sighting);
        double s = Math.sin(sighting);

        double x = pos.getX() + c * pic.ftcPose.range;
        double y = pos.getY() + s * pic.ftcPose.range;

        return new Pose2d( x, y, -this.rotation - pic.ftcPose.yaw) ;
    }

}
