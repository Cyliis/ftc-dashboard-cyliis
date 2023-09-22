package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CoolDigitalSensor {

    private DigitalChannel sensor;

    public CoolDigitalSensor(HardwareMap hm, String name){
        this.sensor = hm.get(DigitalChannel.class, name);
        this.sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState(){
        return sensor.getState();
    }
}
