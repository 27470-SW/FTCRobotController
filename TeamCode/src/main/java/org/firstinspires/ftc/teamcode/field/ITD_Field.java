package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "WeakerAccess"})
public class ITD_Field extends Field
{
    public ITD_Field()
    {
        super("PowerPlay");
        setHasVuMarks(false);
    }

    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars: Pt description

    private static final String TAG = " SJH_RFD";

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;

    //Red Route > Left
    //Red (Left/Right) Start Points
    public static final Point2d ROS1 = new Point2d("ROS1", -61.5,  -47);
    public static final Point2d RIS1 = new Point2d("RIS1", -61.5,  -24);

    //Red (Left/Right) Scan Pt's (Image Scan Pt)
    public static final Point2d ROSP = new Point2d("ROSP", -61.45,  -47);
    public static final Point2d RISP = new Point2d("RISP", -61.45,  -24);

    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int STN_LEFT = 0;
    private static final int STN_CNTR = 1;
    private static final int STN_RGHT = 2;

    private static final String ASSET_NAME = "PowerPlay";

    void setImageNames()
    {
       trackableNames.add("BlueStorage");
       trackableNames.add("BlueAllianceWall");
       trackableNames.add("RedStorage");
       trackableNames.add("RedAllianceWall");
    }

    void setImageLocations()
    {
        float[][] TRACKABLE_POS = {
                scaleArr(new float[]{-halfField,  oneAndHalfTile, IMAGE_Z}),
                scaleArr(new float[]{ halfTile,   halfField,      IMAGE_Z}),
                scaleArr(new float[]{-halfField, -oneAndHalfTile, IMAGE_Z}),
                scaleArr(new float[]{ halfTile,  -halfField,      IMAGE_Z})
        };

        locationsOnField.add(genMatrix(TRACKABLE_POS[0], new float[]{  90.0f,    0.0f,   90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[1], new float[]{  90.0f,    0.0f,    0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[2], new float[]{  90.0f,    0.0f,   90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[3], new float[]{  90.0f,    0.0f,  180.0f}));
    }
}
