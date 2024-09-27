using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

/// <summary>
/// Class used to define useful functions
/// </summary>
public class Utils {

    /// <summary>
    /// Computes the shortest angle distance between two given angles in radians.
    /// The result is thus always in the range [0; PI]
    /// </summary>
    public static float DeltaAngleRadians(float a, float b) {
        return Mathf.PingPong(Mathf.Abs(a - b), Mathf.PI);
    }

    /// <summary>
    /// Computes the shortest (in absolute value) difference between
    /// the two given angles in radians.
    /// The result is thus always in the range[-PI; PI]:
    /// </summary>
    public static float SubstractAngleRadians(float a, float b) {
        float num = Mathf.Repeat(a - b, 2 * Mathf.PI);
        if (num > Mathf.PI) {
            num -= 2 * Mathf.PI;
        }

        return num;
    }

    /// <summary>
    /// Computes the shortest angle between two lines with the given angles
    /// in radians.
    /// The result is thus always in the range [0; PI/2]
    /// </summary>
    public static float LineDeltaAngleRadians(float a, float b) {
        return Mathf.PingPong(Mathf.Abs(a - b), Mathf.PI / 2);
    }

    /// <summary>
    /// Computes the shortest (in absolute value) difference between two 
    /// lines with the given angles in radians.
    /// The result is thus always in the range [-PI/2; PI/2]
    /// </summary>
    public static float LineSubstractAngleRadians(float a, float b) {
        float num = Mathf.Repeat(a - b, Mathf.PI);
        if (num > Mathf.PI/2) {
            num -= Mathf.PI;
        }

        return num;
    }

    /// <summary>
    /// Returns a random gaussian number with the given mean and covariance
    /// </summary>
    public static float RandomGaussian(float mean, float sigma) {
        float u1 = 1 - Random.Range(0, 1f);
        float u2 = 1 - Random.Range(0, 1f);

        // Box-Muller transform:
        return mean + sigma * Mathf.Sqrt(-2 * Mathf.Log(u1)) * Mathf.Cos(2 * Mathf.PI * u2);
    }

    public static string ToString(float value) {
        System.IFormatProvider formatProvider = CultureInfo.InvariantCulture.NumberFormat;
        return value.ToString("F2", formatProvider);
    }

    public static float Round(float value, int digits) {
        float factor = Mathf.Pow(10, digits);
        return Mathf.Round(value * factor) / factor;
    }

    public static Vector3 To3D(float x, float y, float height) {
        return new Vector3(x, height, y);
    }

    public static Vector3 To3D(Vector2 position, float height) {
        return new Vector3(position.x, height, position.y);
    }

    public static Vector2 To2D(Vector3 position) {
        return new Vector2(position.x, position.z);
    }

    public static string ToString(float[] array, float scaleFactor) {
        string result = "[";

        for (int i = 0; i < array.Length; i++) {
            result += array[i] * scaleFactor;
            if (i + 1 < array.Length) result += "; ";
        }

        return result;
    }

    public static string ToString(List<float> array) {
        string result = "[";

        for (int i = 0; i < array.Count; i++) {
            result += array[i];
            if (i + 1 < array.Count) result += "; ";
        }

        return result;
    }
}
