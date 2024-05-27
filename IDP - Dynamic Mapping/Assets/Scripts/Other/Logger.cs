using System.IO;
using UnityEngine;

public class Logger {

    private bool consoleLog;
    private StreamWriter logFile = null;

    public Logger(bool consoleLog) {
        this.consoleLog = consoleLog;
    }

    public Logger(bool consoleLog, string filename) {
        this.consoleLog = consoleLog;
        logFile = File.CreateText(filename);
    }

    public void Log(string message) {
        if(consoleLog)
            Debug.Log(message);

        if(logFile != null)
            logFile.WriteLine(message);
    }

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

    private static float PrincipalDegreeMeasure(float angle) {
        while (angle > Mathf.PI) angle -= 2 * Mathf.PI;
        while (angle < -Mathf.PI) angle += 2 * Mathf.PI;

        return angle * Mathf.Rad2Deg;
    }
}
