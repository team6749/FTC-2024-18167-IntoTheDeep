package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;

class InertialFrameInput  {
    Encoder sensor ;
    double stepDistance;
    int step;

    Vector2d direction, offset;

    public InertialFrameInput(HardwareMap map, String name, double stepDistance, Vector2d direction, Vector2d offset)
    {
        this.sensor = new Encoder(map.get(DcMotorEx.class, name));
        this.stepDistance = stepDistance;
        this.step = 0;
        this.direction = direction;
        this.offset = offset;

    }
    public Vector2d update()
    {
        return new Vector2d( 0, 0 ) ;
    }
} ;

public class InertialFrame {

    InertialFrameInput[] encoders ;

    Pose2d  location ;

    public InertialFrame(InertialFrameInput[] encoders) {
        this.encoders = encoders;
        this.location=  new Pose2d(0, 0, 0 ) ;
    }

    public void set( Pose2d cur )
    {
        this.location = cur ;
        for ( InertialFrameInput in : encoders )
        {
            in.update() ;
        }
    }
    public Pose2d update()
    {
        return new Pose2d( 0, 0, 0 ) ;
    }

    public static class Builder
    {
        private HardwareMap map;
        private double scale ;
        private ArrayList<InertialFrameInput> data = new ArrayList<>();
        
        public Builder(HardwareMap map) {
            this.map = map;
            this.scale = 0 ;
        }
        public Builder(HardwareMap map, double ascale )
        {
            this.map = map ;
            this.scale = ascale ;
        }

        public Builder addInput(String s, double scale, double locx, double locy, double dirx, double diry) {
            data.add(new InertialFrameInput(map, s, this.scale, new Vector2d(locx, locy), new Vector2d(dirx, diry)));
            return this;
        }
        public Builder addInput(String s, double locx, double locy, double dirx, double diry)
        {
            return addInput( s, scale, locx, locy, dirx, diry ) ;
        }
        /**
         * Create an {@link InertialFrame} object from the specified inputs
         * @return an {@link InertialFrame} object
         */
        public InertialFrame build()
        {
            return new InertialFrame(data.toArray(new InertialFrameInput[0]));
        }
    }

}
