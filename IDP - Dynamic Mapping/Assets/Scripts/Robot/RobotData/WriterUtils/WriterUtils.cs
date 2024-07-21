using System.IO;
using UnityEngine;

public class WriterUtils {
    public static string CreateDirectory(string path, string name) {
        int folder_id = Directory.GetDirectories(path).Length;
        string folder_name = path + "/" + name + "_" + folder_id;

        // Make sure to create a new folder:
        while (Directory.Exists(folder_name)) {
            folder_id++;
            folder_name = path + "/" + name + "_" + folder_id;
        }

        // Create the directory:
        Directory.CreateDirectory(folder_name);

        return folder_name;
    }

    public static void SaveConfig(RobotSetup config, string folder) {
        string data = JsonUtility.ToJson(config);

        StreamWriter file = File.CreateText(folder + "/config.json");
        file.Write(data);
        file.Flush();
        file.Close();
    }

    public static void SaveData(RobotFrame frame, string folder) {
        string data = JsonUtility.ToJson(frame);

        StreamWriter file = File.CreateText(folder + "/frame_" + frame.frame_number + ".json");
        file.Write(data);
        file.Flush();
        file.Close();
    }
}