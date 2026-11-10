package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Lightweight variable logger.
 * Writes variables to a CSV file in this format:
 * timestamp, variableName, value
 *
 * Files are stored under /sdcard/FIRST/ on the Robot Controller.
 */
public class VariableLogger {

    private PrintWriter writer;
    private final SimpleDateFormat timeFormat =
            new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);
    private final String fileName;
    private boolean enabled = true;

    /**
     * @param fileName log file name, e.g. "drive_log.csv"
     */
    public VariableLogger(String fileName) {
        this.fileName = fileName;
        openFile();
    }

    private void openFile() {
        try {
            File logDir = new File("/sdcard/FIRST/");
            if (!logDir.exists()) {
                logDir.mkdirs();
            }

            File logFile = new File(logDir, fileName);
            writer = new PrintWriter(new FileWriter(logFile, true)); // append mode
            writer.println("timestamp,variable,value");
            writer.flush();
        } catch (IOException e) {
            enabled = false;
            e.printStackTrace();
        }
    }

    /**
     * Log a single variable.
     *
     * @param variableName variable name
     * @param value        variable value; supports int / double / boolean / String
     */
    public void log(String variableName, Object value) {
        if (!enabled || writer == null) return;

        String timestamp = timeFormat.format(new Date());
        writer.printf("%s,%s,%s%n", timestamp, variableName, String.valueOf(value));
        writer.flush(); // flush immediately so data is not lost if OpMode stops
    }

    /**
     * Log multiple variables in one call.
     */
    public void log(String[] names, Object[] values) {
        if (!enabled || writer == null) return;
        if (names.length != values.length) return;

        String timestamp = timeFormat.format(new Date());
        for (int i = 0; i < names.length; i++) {
            writer.printf("%s,%s,%s%n", timestamp, names[i], String.valueOf(values[i]));
        }
        writer.flush();
    }

    /** Call this at the end of the OpMode to close the file. */
    public void close() {
        if (writer != null) {
            writer.flush();
            writer.close();
        }
    }
}
