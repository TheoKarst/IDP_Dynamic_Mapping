using UnityEngine;

public class Utils {
    // The number of digits we print after the comma, in scientific notation:
    const int PRINT_DIGITS = 3;

    // Computes the shortest difference between two given angles in radians.
    // The result is thus always in the range [0; PI]:
    public static float DeltaAngleRadians(float a, float b) {
        return Mathf.Abs(SubstractAngleRadians(a, b));
    }

    // Compute the difference between the two given angles in radians, and
    // keep the result in the range[-PI; PI]:
    public static float SubstractAngleRadians(float a, float b) {
        float num = Mathf.Repeat(a - b, 2 * Mathf.PI);
        if (num > Mathf.PI) {
            num -= 2 * Mathf.PI;
        }

        return num;
    }

    // For lines, the difference between their orientation is always
    // in the range [-PI/2; PI/2]:
    public static float SubstractLineAngleRadians(float a, float b) {
        float num = Mathf.Repeat(a - b, Mathf.PI);
        if (num > Mathf.PI/2) {
            num -= Mathf.PI;
        }

        return num;
    }



    public static string ScientificNotation(float value) {
        return ScientificNotation(value, PRINT_DIGITS);
    }

    public static string ScientificNotation(float value, int printDigits) {
        float absValue = Mathf.Abs(value);
        if (absValue < Mathf.Epsilon)
            return "0";

        int digits = Mathf.FloorToInt(Mathf.Log10(absValue));
        float factor = Mathf.Pow(10, printDigits - digits);
        string result = (Mathf.Round(value * factor) / Mathf.Pow(10, printDigits)).ToString();
        if (digits != 0)
            result += "e" + digits;

        return result;
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

    public static string ToString(float[] array) {
        string result = "[";

        for (int i = 0; i < array.Length; i++) {
            result += array[i];
            if (i + 1 < array.Length) result += "; ";
        }

        return result;
    }
}
