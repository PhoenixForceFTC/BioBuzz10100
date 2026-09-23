package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/** Finds the horizontal center of a BIOBUZZ cell from Limelight fiducial poses. */
final class HiveTagTracker {
    private static final int FIRST_ID = 30;
    private static final int LAST_ID = 45;
    private static final double[] MEMBER_X_INCHES = {-6.5, -2.75, 2.75, 6.5};
    // SDK 12.0 BioBuzz cluster metadata places the four member centers at these X values.
    // The cluster X origin is at the center of the cell opening.
    private static final String[] CELL_NAMES = {
            "RED opposite audience", "RED audience", "BLUE audience", "BLUE opposite audience"
    };

    static final class Cell {
        final int index;
        final String name;
        final double bearingDegrees;
        final int visibleTags;
        final String ids;

        Cell(int index, double bearingDegrees, int visibleTags, String ids) {
            this.index = index;
            this.name = CELL_NAMES[index];
            this.bearingDegrees = bearingDegrees;
            this.visibleTags = visibleTags;
            this.ids = ids;
        }
    }

    private HiveTagTracker() { }

    static String identify(int id) {
        return id >= FIRST_ID && id <= LAST_ID
                ? CELL_NAMES[(id - FIRST_ID) / 4] : "unknown";
    }

    static List<Cell> visibleCells(LLResult result) {
        List<Cell> cells = new ArrayList<>();
        // getStaleness() is in milliseconds, measured on the Control Hub.
        if (result == null || !result.isValid() || result.getStaleness() > 250) {
            return cells;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return cells;
        }
        for (int group = 0; group < CELL_NAMES.length; group++) {
            double sumX = 0;
            double sumRange = 0;
            double sumMemberX = 0;
            double sumMemberSquare = 0;
            double sumCross = 0;
            int count = 0;
            StringBuilder ids = new StringBuilder();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id < FIRST_ID + group * 4 || id >= FIRST_ID + (group + 1) * 4
                        || fiducial.getTargetPoseCameraSpace() == null) {
                    continue;
                }
                Position p = fiducial.getTargetPoseCameraSpace()
                        .getPosition().toUnit(DistanceUnit.INCH);
                double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                double tx = fiducial.getTargetXDegreesNoCrosshair();
                if (!Double.isFinite(range) || !Double.isFinite(tx)
                        || range < 4 || Math.abs(tx) > 45) {
                    continue;
                }
                double memberX = MEMBER_X_INCHES[(id - FIRST_ID) % 4];
                // Limelight 2026 and 2027 firmware use different 3-D axes.
                // tx without crosshair is a stable image bearing in either version.
                double observedX = Math.tan(Math.toRadians(tx)) * range;
                sumX += observedX;
                sumRange += range;
                sumMemberX += memberX;
                sumMemberSquare += memberX * memberX;
                sumCross += memberX * observedX;
                if (ids.length() > 0) ids.append(',');
                ids.append(id);
                count++;
            }
            if (count == 0) continue;

            // Fit observed tag X against the known positions in the four-tag row.
            // With one tag, use its known offset; with more tags, the fitted slope
            // compensates for foreshortening when viewing the cluster obliquely.
            double slope = 1.0;
            double denominator = count * sumMemberSquare - sumMemberX * sumMemberX;
            if (denominator > 1e-6) {
                double fitted = (count * sumCross - sumMemberX * sumX) / denominator;
                if (Math.abs(fitted) > 0.35 && Math.abs(fitted) < 1.25) {
                    slope = fitted;
                }
            } else {
                // An inverted cell presents the four tags in reverse screen order.
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (ids.toString().equals(Integer.toString(fiducial.getFiducialId()))
                            && fiducial.getTargetPoseCameraSpace() != null
                            && Math.abs(fiducial.getTargetPoseCameraSpace().getOrientation()
                                    .getRoll(AngleUnit.DEGREES)) > 90) {
                        slope = -1.0;
                    }
                }
            }
            double centerX = (sumX - slope * sumMemberX) / count;
            double centerRange = sumRange / count;
            // This is a turning target only. The camera's 75-degree elevation
            // changes range, but zero horizontal bearing still means centered.
            double bearing = Math.toDegrees(Math.atan2(centerX, centerRange));
            cells.add(new Cell(group, bearing, count, ids.toString()));
        }
        return cells;
    }

    static Cell nearestVisible(List<Cell> cells) {
        Cell best = null;
        for (Cell cell : cells) {
            if (best == null || Math.abs(cell.bearingDegrees) < Math.abs(best.bearingDegrees)) {
                best = cell;
            }
        }
        return best;
    }

    static Cell find(List<Cell> cells, int index) {
        for (Cell cell : cells) {
            if (cell.index == index) return cell;
        }
        return null;
    }
}
