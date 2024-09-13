import json

def load_json(path : str):
        """ Loads a json file and returns data as a dictionnary """

        with open(path, "r") as f:
            return json.load(f)
    