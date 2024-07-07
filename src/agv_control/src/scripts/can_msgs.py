class CanRxMsg:
    def __init__(self):
        self.StdId = 0  # Standard identifier, value between 0 to 0x7FF
        self.ExtId = 0  # Extended identifier, value between 0 to 0x1FFFFFFF
        self.IDE = 0  # Identifier type for the message that will be received
        self.RTR = 0  # Type of frame for the received message
        self.DLC = 0  # Length of the frame that will be received, value between 0 to 8
        self.Data = [0]*8  # Data to be received, each element ranges from 0 to 0xFF
        self.FMI = 0  # Index of the filter the message stored in the mailbox passes through, value between 0 to 0xFF