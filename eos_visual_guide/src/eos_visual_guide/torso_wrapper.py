import fetch_api
import sys


class TorsoWrapper:
    def __init__(self):
        self._torso = fetch_api.Torso()
        sys.stdout.write("Setting torso height for EOS...")
        sys.stdout.flush()
        self._torso.set_height(0.2)
        print "done!"
