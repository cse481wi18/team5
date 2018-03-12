import gripper_wrapper
from .user_input import UserInput, UserCommand, COMMAND_SAVE, COMMAND_EXIT, COMMAND_EXECUTE, COMMAND_LIST, COMMAND_SHOW
import saver
import constants


def _print_choose_frame_text(ar_tags, default_frame=gripper_wrapper.DEFAULT_FRAME):
    """
    Prints the helper text to choose a frame
    :param ar_tags: array of AR tags with an `id` field
    :return:
    """
    choose_frame_text = "Choose a frame:\n"
    choose_frame_text += ("-1: %s; " % default_frame)
    choose_frame_text += str([x.id for x in ar_tags])

    print choose_frame_text


def _print_choose_action_text(locations, grippers):
    choose_action_text = "Locations:\n"
    choose_action_text += str(locations)
    choose_action_text += "\n"
    choose_action_text += "Grippers:\n"
    choose_action_text += str(grippers)
    choose_action_text += "\n"

    print choose_action_text


class Cli(UserInput):
    def __init__(self):
        UserInput.__init__(self)

        print "Welcome to the EOS Command Line Interface!"
        print "Loading subscribers and publishers..."

    def get_command(self):
        """
        Parsing text input into a UserCommand object.
        :return:
        """
        while True:
            text = raw_input('> ').split()
            try:
                if text[0] == "exit":
                    return UserCommand(COMMAND_EXIT)
                if text[0] == "save":
                    return UserCommand(COMMAND_SAVE, {
                        "type": text[1],
                        "name": text[2],
                    })
                elif text[0] == "list":
                    return UserCommand(COMMAND_LIST)
                elif text[0] == "show":
                    return UserCommand(COMMAND_SHOW, {
                        "type": text[1],
                        "name": text[2],
                    })
                elif text[0] == "execute":
                    return UserCommand(COMMAND_EXECUTE, {
                        "type": text[1],
                        "name": text[2],
                    })

            except IndexError:
                print "Illegal # of arguments"

    def get_frame_id(self, ar_tags):
        """
        Returns the frame id from the ar_tags
        :param ar_tags:
        :return:
        """
        frame_string = None
        while not frame_string:
            _print_choose_frame_text(ar_tags)
            tag_id = int(raw_input('>> ').split()[0])
            if tag_id in [x.id for x in ar_tags] or tag_id is -1:
                frame_string = gripper_wrapper.DEFAULT_FRAME if tag_id is -1 else "ar_marker_%d" % tag_id
            else:
                print "illegal frame"
        return frame_string

    def get_actions(self, locations, grippers):
        actions = []
        while True:
            _print_choose_action_text(locations, grippers)
            entries = raw_input('>> ').split()
            if not len(entries):
                return actions
            # TODO ensure len 2
            # TODO ensure that entries are valid
            actions.append(entries)
