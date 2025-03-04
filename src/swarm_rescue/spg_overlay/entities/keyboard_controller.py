import arcade


class KeyboardController:
    """
    The KeyboardController class is responsible for handling keyboard input and
    converting it into commands for controlling a robot. It keeps track of the
    current command values for forward movement, lateral movement,
    rotation, and the grasper.
    """

    def __init__(self, number_drones):
        # A dictionary that stores the current command values for forward movement, lateral movement, rotation,
        # and the grasper.
        self._command = {"forward": 0.0,
                         "lateral": 0.0,
                         "rotation": 0.0,
                         "grasper": 0}
        
        self.identifier = 0
        self.number_drones = number_drones

    # def on_key_press(self, key, modifiers, commands: Dict[Union[str, Controller], Command]):
    def on_key_press(self, key, modifiers):
        """Called whenever a key is pressed. Updates the command values based
        on the pressed key."""
        if self._command:

            if key == arcade.key.UP:
                self._command["forward"] = 0.2
            elif key == arcade.key.DOWN:
                self._command["forward"] = -0.2

            if not modifiers & arcade.key.MOD_SHIFT:
                if key == arcade.key.LEFT:
                    self._command["rotation"] = 0.2
                elif key == arcade.key.RIGHT:
                    self._command["rotation"] = -0.2
            else:
                if key == arcade.key.LEFT:
                    self._command["lateral"] = 0.2
                elif key == arcade.key.RIGHT:
                    self._command["lateral"] = -0.2

            if key == arcade.key.W:
                self._command["grasper"] = 1

            if key == arcade.key.I:
                self.identifier += 1
                if self.identifier % self.number_drones == 0:
                    self.identifier = 0

    # def on_key_release(self, key, modifiers, commands: Dict[Union[str, Controller], Command]):
    def on_key_release(self, key, modifiers):
        """Called whenever a key is released. Resets the command values based
        on the released key."""
        if self._command:

            if key == arcade.key.UP:
                self._command["forward"] = 0
            elif key == arcade.key.DOWN:
                self._command["forward"] = 0

            if key == arcade.key.LEFT:
                self._command["lateral"] = 0
                self._command["rotation"] = 0
            elif key == arcade.key.RIGHT:
                self._command["lateral"] = 0
                self._command["rotation"] = 0

            if key == arcade.key.W:
                self._command["grasper"] = 0

    def control(self):
        """Returns the current command values."""
        return self._command
