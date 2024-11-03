package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.FieldReferenceLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;

public class FieldReferenceLibrary {

    public FieldReference[] references;
    private FieldReference  zero;

    public FieldReferenceLibrary(FieldReference[] references) {
        this.references = references;
        this.zero = new FieldReference( 0, 0, 0, 0 ) ;
    }

    private FieldReference getTagByID(int id) {
        for (FieldReference ref : references) {
            if (ref.tag_id == id) {
                return ref;
            }
        }
        return zero;
    }

    public Pose2d locate ( AprilTagDetection pic )
    {
        return getTagByID(pic.id).locate( pic ) ;
    }

    public static class Builder
    {
        private ArrayList<FieldReference> data = new ArrayList<>();

        public Builder addTag(int id, float posX, float posY, int angle ) {
            data.add(new FieldReference(id, posX, posY, angle));
            return this;
        }
        /**
         * Create an {@link FieldReferenceLibrary} object from the specified tags
         * @return an {@link FieldReferenceLibrary} object
         */
        public FieldReferenceLibrary build()
        {
            return new FieldReferenceLibrary(data.toArray(new FieldReference[0]));
        }
    }

    public double cameraScan(Pose2d pose) {
        double theta= Math.toRadians(pose.getHeading()) ;
        Vector2d cnorm = Vector2d.polar( 1, theta + Math.PI);

        FieldReference tagref = zero;

        double maxDistance = 600000;

        for (FieldReference ref : references) {
            Vector2d delt = ref.pos.minus(pose.vec()) ;
            double n = cnorm.dot(delt);

            if (n > 0) {
                double d2 = delt.dot(delt) ;
                if (d2 < maxDistance) {
                    maxDistance = d2;
                    tagref = ref;
                }
            }
        }

        if (tagref.tag_id != 0) {
            Vector2d delt = tagref.pos.minus(pose.vec()) ;
            double angle = Math.atan2(delt.getY(), delt.getX());
            double heading = Math.toDegrees(angle)  - pose.getHeading() -180 ;
            while ( heading > 180 ) { heading -= 360 ; }
            while ( heading < -180 ) { heading += 360 ; }
            return heading ;
        }
        return 0.0;
    }
}