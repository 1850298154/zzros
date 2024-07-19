import pprint

from src            import tracker

class Base_Parser:
    def __init__(self) -> None:
        pass

    def initialize(self) -> None:
        pass

    def start(self) -> None:
        pass

    # @staticmethod
    # def mount_dict2obj(obj, kwargs):
    def mount_dict2obj(self, obj, kwargs):
        keys_set = {key for key in kwargs.keys()}

        valid_fields = keys_set # {'name', 'age', 'city'}
        for key, value in kwargs.items():
            if key in valid_fields:
                setattr(obj, key, value)
            else:
                tracker.debug_tracker.instance.current_position_print()
                tracker.violation_detector.instance.raise_key_error(
                    "JSON not in Class : " + str(key)
                )
                # print("kwargs")
                # pprint.pprint(kwargs)
                # raise AttributeError(
                #             f"Invalid attribute: {key}"
                #             )
        return obj
