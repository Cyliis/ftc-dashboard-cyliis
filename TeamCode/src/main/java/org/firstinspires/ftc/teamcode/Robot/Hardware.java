package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;

public class Hardware {
    public DcMotorEx mch0, mch1, mch2, mch3;
    public DcMotorEx meh0, meh1, meh2, meh3;

    public Servo sch0, sch1, sch2, sch3, sch4, sch5;
    public Servo seh0, seh1, seh2, seh3, seh4, seh5;

    public CoolIMU imu;

    public Hardware(HardwareMap hm){
        mch0 = hm.get(DcMotorEx.class, "ch0");
        mch1 = hm.get(DcMotorEx.class, "ch1");
        mch2 = hm.get(DcMotorEx.class, "ch2");
        mch3 = hm.get(DcMotorEx.class, "ch3");

        meh0 = hm.get(DcMotorEx.class, "eh0");
        meh1 = hm.get(DcMotorEx.class, "eh1");
        meh2 = hm.get(DcMotorEx.class, "eh2");
        meh3 = hm.get(DcMotorEx.class, "eh3");

        sch0 = hm.get(Servo.class, "eh0");
        sch1 = hm.get(Servo.class, "eh1");
        sch2 = hm.get(Servo.class, "eh2");
        sch3 = hm.get(Servo.class, "eh3");
        sch4 = hm.get(Servo.class, "eh4");
        sch5 = hm.get(Servo.class, "eh5");

        seh0 = hm.get(Servo.class, "eh0");
        seh1 = hm.get(Servo.class, "eh1");
        seh2 = hm.get(Servo.class, "eh2");
        seh3 = hm.get(Servo.class, "eh3");
        seh4 = hm.get(Servo.class, "eh4");
        seh5 = hm.get(Servo.class, "eh5");

        imu = new CoolIMU(hm);
    }
}
