using System.IO;
using UnityEngine;

/// <summary>
/// Simple script to log data to Unity console or to a file
/// </summary>
public class Logger {
    
    private bool consoleLog;
    private StreamWriter logFile = null;

    /// <summary>
    /// Creates a logger
    /// </summary>
    /// <param name="consoleLog">If the data should be logged to Unity console</param>
    public Logger(bool consoleLog) {
        this.consoleLog = consoleLog;
    }

    /// <summary>
    /// Creates a logger
    /// </summary>
    /// <param name="consoleLog">If the data should be logged to Unity console</param>
    /// <param name="filename">File in which data will be logged</param>
    public Logger(bool consoleLog, string filename) {
        this.consoleLog = consoleLog;
        logFile = File.CreateText(filename);
    }

    /// <summary>
    /// Loggs the given message (in Unity console and/or in a file)
    /// </summary>
    public void Log(string message) {
        if(consoleLog)
            Debug.Log(message);

        if(logFile != null)
            logFile.WriteLine(message);
    }

    /// <summary>
    /// Loggs the given message with a timestamp (in Unity console and/or in a file)
    /// </summary>
    public void TimeLog(string message) {
        Log(Time.time + ": " + message);
    }

    public void Log(params VehicleState[] states) {
        if (consoleLog) {
            string text = Time.time + ": ";
            for (int i = 0; i < states.Length; i++) {
                text += states[i].ToString();
                if (i + 1 < states.Length) text += "; ";
            }
            Debug.Log(text);
        }

        if(logFile != null) {
            string text = Time.time + ";";
            for (int i = 0; i < states.Length; i++) {
                text += states[i].x + ";" 
                    + states[i].y + ";" 
                    + PrincipalDegreeMeasure(states[i].phi);

                if (i + 1 < states.Length) text += "; ";
            }
            logFile.WriteLine(text);
        }
    }

    /// <summary>
    /// Returns the principal degree measure of an angle (between -180 and 180°)
    /// </summary>
    /// <param name="angle">Angle in radians</param>
    private static float PrincipalDegreeMeasure(float angle) {
        return Mathf.Rad2Deg * Mathf.Repeat(angle, 2 * Mathf.PI) - 180;
    }
}
