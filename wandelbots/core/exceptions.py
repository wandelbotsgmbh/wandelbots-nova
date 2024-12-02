class ControllerNotFoundException(Exception):
    def __init__(self, controller: str = None):
        super().__init__(f"Controller {controller} not found.")
