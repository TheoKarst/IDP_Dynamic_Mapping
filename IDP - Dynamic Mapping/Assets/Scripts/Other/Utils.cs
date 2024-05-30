using UnityEngine;

public class Utils
{
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
}
