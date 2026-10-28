package org.firstinspires.ftc.teamcode;

/** TU06 Section 9 nominal dimensions. Inch labels are the common conversion source.
 * Derived longitudinal positions retain the approved symmetric half-length projection;
 * they are not CAD-verified pivot/surface transforms. See doc/FieldReference.md.
 */
public final class FieldGeometry {
    private FieldGeometry() { }
    public static final double METERS_PER_INCH = .0254;
    public static final double FIELD_SIZE_INCHES = 144;
    public static final double FIELD_HALF_WIDTH_METERS = FIELD_SIZE_INCHES * METERS_PER_INCH / 2;
    public static final double HIVE_CENTER_SPACING_INCHES = 25.5;
    public static final double HIVE_PIVOT_HEIGHT_INCHES = 43.95;
    public static final double HIVE_LENGTH_INCHES = 42.91;
    public static final double HIVE_CELL_GAP_INCHES = 18.84;
    public static final double HIVE_CELL_DEPTH_INCHES = 12.04;
    public static final double HIVE_TILT_DEG = 30;
    public static final double OPENING_PLANE_DEG = 90 - HIVE_TILT_DEG;
    public static final double OPENING_WIDTH_INCHES = 20;
    public static final double OPENING_SHOULDER_HEIGHT_INCHES = 7.61;
    public static final double OPENING_PEAK_HEIGHT_INCHES = 14;
    public static final double RAISED_OPENING_BOTTOM_HEIGHT_INCHES = 53.5;
    public static final double RAISED_OPENING_TOP_HEIGHT_INCHES = 65.6;
    public static final double HALF_HIVE_LENGTH_METERS = HIVE_LENGTH_INCHES * METERS_PER_INCH / 2;
    public static final double RAISED_OPENING_BOTTOM_HEIGHT_METERS = RAISED_OPENING_BOTTOM_HEIGHT_INCHES * METERS_PER_INCH;
    // Chosen aim point is the midpoint of the drawing's top/bottom heights, not the pentagon centroid.
    public static final double OPENING_AIM_HEIGHT_METERS = (RAISED_OPENING_BOTTOM_HEIGHT_INCHES
            + RAISED_OPENING_TOP_HEIGHT_INCHES) * METERS_PER_INCH / 2;
    public static final double NOMINAL_FRONT_Y_METERS = HALF_HIVE_LENGTH_METERS * Math.cos(Math.toRadians(HIVE_TILT_DEG));
    public static final double OPENING_Y_OFFSET_METERS = NOMINAL_FRONT_Y_METERS
            - (OPENING_AIM_HEIGHT_METERS - RAISED_OPENING_BOTTOM_HEIGHT_METERS) / Math.tan(Math.toRadians(OPENING_PLANE_DEG));
}
