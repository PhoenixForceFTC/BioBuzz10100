package org.firstinspires.ftc.teamcode;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.Properties;

/** Versioned persistent notebook, with an independent parser for desktop/storage verification.
 * Exports are explicitly unvalidated session data, never automatically loaded as a firing profile.
 */
public final class ShotCalibrationExport {
    private ShotCalibrationExport() { }
    public static final String STATUS = "SESSION_UNVALIDATED_STATIONARY_ONLY";
    public static final class Data {
        public final String geometry, status;
        public final ShotCalibration.Sample[] samples;
        public final ShotProfile.Row[] rows;
        Data(String geometry, String status, ShotCalibration.Sample[] samples, ShotProfile.Row[] rows) {
            this.geometry=geometry; this.status=status; this.samples=samples; this.rows=rows;
        }
    }
    public static void write(Writer writer, ShotCalibration notebook, String geometry) throws IOException {
        if (geometry == null || geometry.trim().isEmpty()) throw new IllegalArgumentException("Geometry identity required");
        Properties p=new Properties();
        p.setProperty("format","BioBuzz-shot-notebook-v1");
        p.setProperty("geometry",geometry); p.setProperty("calibration_status",STATUS);
        p.setProperty("units","distance=m,hood=deg_down_from_home,rpm=motor_rev/min,flight=s");
        p.setProperty("distance_reference","horizontal_muzzle_to_opening");
        p.setProperty("mount","pivot_forward_mm=50.68184865,camera_from_pivot_mm=-99.53073974,camera_height_mm=335,pitch_up_deg=30");
        ShotCalibration.Sample[] samples=notebook.samples(); ShotProfile.Row[] rows=notebook.rows();
        p.setProperty("sample_count",Integer.toString(samples.length)); p.setProperty("row_count",Integer.toString(rows.length));
        for (int i=0;i<samples.length;i++) {
            ShotCalibration.Sample s=samples[i];
            p.setProperty("sample."+i,s.distanceMeters+","+s.hoodDeg+","+s.rpm+","+s.hit);
        }
        for (int i=0;i<rows.length;i++) {
            ShotProfile.Row r=rows[i]; p.setProperty("row."+i,r.distance+","+r.hoodDeg+","+r.rpm+","+r.flightSeconds);
        }
        p.store(writer,"BioBuzz session calibration; columns distance_m,hood_travel_deg,motor_rpm,hit_or_flight_s");
        writer.flush();
    }
    public static Data read(Reader reader) throws IOException {
        Properties p=new Properties(); p.load(reader);
        try {
            if (!"BioBuzz-shot-notebook-v1".equals(p.getProperty("format")) || !STATUS.equals(p.getProperty("calibration_status"))
                    || p.getProperty("geometry","").isEmpty() || !"distance=m,hood=deg_down_from_home,rpm=motor_rev/min,flight=s".equals(p.getProperty("units")))
                throw new IllegalArgumentException("Unknown notebook format / units / status");
            int sampleCount=Integer.parseInt(p.getProperty("sample_count")), rowCount=Integer.parseInt(p.getProperty("row_count"));
            if (sampleCount < 0 || sampleCount > 100000 || rowCount < 0 || rowCount > sampleCount) throw new IllegalArgumentException("Invalid counts");
            ShotCalibration.Sample[] samples=new ShotCalibration.Sample[sampleCount]; ShotProfile.Row[] rows=new ShotProfile.Row[rowCount];
            for (int i=0;i<sampleCount;i++) {
                String[] v=p.getProperty("sample."+i).split(",");
                double d=Double.parseDouble(v[0]), h=Double.parseDouble(v[1]), rpm=Double.parseDouble(v[2]);
                if (v.length != 4 || !(v[3].equals("true") || v[3].equals("false"))) throw new IllegalArgumentException("Invalid hit");
                ShotCalibration validation=new ShotCalibration(); validation.configure(d/.0254,rpm,h);
                samples[i]=new ShotCalibration.Sample(d,rpm,h,Boolean.parseBoolean(v[3]));
            }
            double previous=0;
            for (int i=0;i<rowCount;i++) {
                String[] v=p.getProperty("row."+i).split(",");
                if (v.length != 4) throw new IllegalArgumentException("Invalid row");
                double d=Double.parseDouble(v[0]), h=Double.parseDouble(v[1]), rpm=Double.parseDouble(v[2]), flight=Double.parseDouble(v[3]);
                ShotCalibration validation=new ShotCalibration(); validation.configure(d/.0254,rpm,h);
                if (d <= previous || (!Double.isNaN(flight) && (!Double.isFinite(flight) || flight <= 0))) throw new IllegalArgumentException("Invalid row ordering / flight");
                rows[i]=new ShotProfile.Row(d,h,rpm,flight); previous=d;
            }
            return new Data(p.getProperty("geometry"),p.getProperty("calibration_status"),samples,rows);
        } catch (RuntimeException e) { throw new IOException("Invalid calibration notebook",e); }
    }
    public static File save(File directory, ShotCalibration notebook, String geometry) throws IOException {
        if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create " + directory);
        File temporary=File.createTempFile("shot-session-",".tmp",directory);
        File destination=new File(directory,temporary.getName().replace(".tmp",".properties"));
        boolean saved=false;
        try {
            try (Writer writer=new OutputStreamWriter(new FileOutputStream(temporary),StandardCharsets.UTF_8)) { write(writer,notebook,geometry); }
            try (Reader reader=new InputStreamReader(new FileInputStream(temporary),StandardCharsets.UTF_8)) { read(reader); }
            if (!temporary.renameTo(destination)) throw new IOException("Cannot finalize " + destination);
            saved=true; return destination;
        } finally { if (!saved && temporary.exists() && !temporary.delete()) temporary.deleteOnExit(); }
    }
}
