import ntcore

from components.led import LEDs, LEDStates
from magicbot import feedback


class Dashboard:
    leds: LEDs

    def __init__(self):
        """
        Initializes a network tables server with a client-side python script that runs the pygame dashboard
        """
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.datatable = self.nt_instance.getTable("dashboard")

        self.current_selected_node = ""
        self.node_receiver = self.datatable.getStringTopic("Node Selector").subscribe(
            "Initial Value"
        )

        self.auton_reciever = self.datatable.getStringTopic("Auton Path").subscribe(
            "Initial Value"
        )
        self.current_selected_auton = ""

    def execute(self) -> None:
        """
        Called every periodic cycle to update the client with confidence levels and update the robot
        with the current selected node
        """
        reading = self.node_receiver.get()

        if (
            reading != self.current_selected_node
            and reading != ""
            and reading != "Initial Value"
        ):
            self.leds.state = LEDStates.NODE_SWITCH

            self.current_selected_node = reading

    def get_node(self) -> str:
        """
        Gets the node currently selected on the pygame dashboard

        Returns:
            str: Current node in the form "A2", "H3", "C4"
        """
        return self.current_selected_node

    def get_custom_auton(self) -> str:
        """
        Returns the current auto selected by the dashboard

        Returns:
            str: the current auton route in the form "A,B,C,D"
        """
        return self.auton_reciever.get()

    @feedback
    def check_auton(self) -> bool:
        """
        Checks if an auto has been selected or not

        Returns:
            bool: true if auton selected by dashboard does not equal default auto
        """
        return self.auton_reciever.get() != "Initial Value"
