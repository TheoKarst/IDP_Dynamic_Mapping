using System.IO;
using UnityEngine;

/// <summary>
/// Useful class for recording data into files
/// </summary>

public class WriterUtils {

    /// <summary>
    /// Creates a directory at the given location and with the given name. This
    /// function makes sure that the created directory is unique by adding a unique
    /// id to the name of the folder
    /// </summary>
    /// <param name="path">Path where the directory should be created</param>
    /// <param name="name">Prefix of the name of the directory</param>
    /// <returns>Full name of the unique directory that was created (path + name + unique_id)</returns>
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

    /// <summary>
    /// Saves the given config in the given folder, in a json file named config.json
    /// </summary>
    public static void SaveConfig(RobotSetup config, string folder) {
        string data = JsonUtility.ToJson(config);

        StreamWriter file = File.CreateText(folder + "/config.json");
        file.Write(data);
        file.Flush();
        file.Close();
    }

    /// <summary>
    /// Saves the given frame in the given folder, in a json file named frame_[frame_number].json
    /// </summary>
    public static void SaveData(RobotFrame frame, string folder) {
        string data = JsonUtility.ToJson(frame);

        StreamWriter file = File.CreateText(folder + "/frame_" + frame.frame_number + ".json");
        file.Write(data);
        file.Flush();
        file.Close();
    }
}