package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.testopmode.TestOpMode;

import java.util.ArrayList;

public class TestFieldVersatilityOpMode extends TestOpMode {
    TestDashboardInstance dashboard;
    public ArrayList<GPose> robotPos = new ArrayList<>();

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    public TestFieldVersatilityOpMode() {
        super("TestFieldVersatilityOpMode");
    }

    @Override
    protected void init() {
        loops++;
        dashboard = TestDashboardInstance.getInstance();
        dashboard.core.addConfigVariable("Test", "ORIGIN_HEADING_OFFSET", new ValueProvider<Double>() {
            private double x;

            @Override
            public Double get() {
                return x;
            }

            @Override
            public void set(Double value) {
                x = value;
            }
        });
        dashboard.core.addConfigVariable("Test", "RED_ALLIANCE", new ValueProvider<Boolean>() {
            private boolean red;

            @Override
            public Boolean get() {
                return red;
            }

            @Override
            public void set(Boolean value) {
                red = value;
            }
        });
    }
    public double loops = 0;

    @Override
    protected void loop() throws InterruptedException {

        //draw the field overlay - supply false if we want to suppress the default field image
        //TelemetryPacket packet = new TelemetryPacket(false);
        TelemetryPacket packet = new TelemetryPacket();

            robotPos = new ArrayList<>();
            if(loops<2*Math.PI)
            for (double i = 0; i < 2 * Math.PI; i += 0.01)
                robotPos.add(new GPose(10 * i, 10 * Math.cos(i), loops));
            packet.fieldOverlay().strokeActualPath(robotPos);


        dashboard.sendTelemetryPacket(packet);
        Thread.sleep(10);
        loops+=0.01;
    }

}
