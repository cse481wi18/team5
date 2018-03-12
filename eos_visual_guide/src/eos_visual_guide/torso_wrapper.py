import fetch_api


class TorsoWrapper:
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._torso.set_height(0.2)
