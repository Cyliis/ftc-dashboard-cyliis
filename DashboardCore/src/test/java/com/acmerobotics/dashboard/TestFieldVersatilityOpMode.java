package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.testopmode.TestOpMode;

public class TestFieldVersatilityOpMode extends TestOpMode {
    TestDashboardInstance dashboard;
    public static double AMPLITUDE = 1;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.25;
    public static double ORIGIN_OFFSET_X = 0;
    public static double ORIGIN_OFFSET_Y = 12 * 6;
    public static double ORIGIN_ZEROHEADING = Math.PI / 2;
    public static boolean RED_ALLIANCE = true;
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;
    public static String ALTIMGSRC = "https://upload.wikimedia.org/wikipedia/commons/4/45/Football_field.svg";
    //public static String ALTIMGSRC = "dist/assets/play_arrow.95e2d7e4.svg";
    public static double ALTIMGX = 0; //try 24
    public static double ALTIMGY = 0; //try 24
    public static double ALTIMGW = 144; //try 48
    public static double ALTIMGH = 144; //try 48
    public static double SCALEX = 1.0;
    public static double SCALEY = 1.0;
    public static int GRID_LINESX = 7; //includes field edges
    public static int GRID_LINESY = 7;
    public static double GRIDX = -24;
    public static double GRIDY = 24;
    public static double GRIDW = 48;
    public static double GRIDH = 48;
    public static double GRID_THETA_DEGREES = 45;
    public static double GRID_PIVOTX = 24;
    public static double GRID_PIVOTY = 24;
    public static boolean GRID_USE_PAGE_FRAME = false;

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

    @Override
    protected void loop() throws InterruptedException {
        System.out.println(Math.sin(System.currentTimeMillis()));
        double time = System.currentTimeMillis() / 1000d;
        double angleAnim = 2 * Math.PI * SPIN_FREQUENCY * time;
        long millis = System.currentTimeMillis();
        double seconds = millis / 1000.0 * SPIN_FREQUENCY;
        double fraction = seconds - (int) seconds;
        angleAnim = 2 * Math.PI * fraction;

        double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double l = SIDE_LENGTH / 2;
        //drawing an orbiting triangle pointing mostly up the X axis to indicate field theta = 15 degrees counter clockwise
        double[] bxPoints = {0, SIDE_LENGTH * 2, 0};
        double[] byPoints = {l, 0, -l};
        //rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
        rotatePoints(bxPoints, byPoints, Math.toRadians(15));
        for (int i = 0; i < 3; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }
        dashboard.addData("x", AMPLITUDE * Math.sin(
                2 * Math.PI * FREQUENCY * (System.currentTimeMillis() / 1000d) + Math.toRadians(PHASE)
        ));
        dashboard.addData("theta", angleAnim);
        dashboard.update();

        //draw the field overlay - supply false if we want to suppress the default field image
        //TelemetryPacket packet = new TelemetryPacket(false);
        TelemetryPacket packet = new TelemetryPacket();

        double start = 0, end = 1;
        double eps = 0.001;
        while (start <= end) {
            packet.fieldOverlay().strokeLine(
                    Math.sin(start)*15,
                    Math.cos(start)*10,
                    Math.sin(start + eps)*10,
                    Math.cos(start + eps)*10
            );
            start += eps;
        }

        dashboard.sendTelemetryPacket(packet);
        Thread.sleep(10);
    }

}
