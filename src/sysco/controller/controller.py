class Controller:
    def __init__(self):
        print("Controller loaded...")

    def get_input(self):
        raise NotImplementedError("Subclasses must implement get_input()")
