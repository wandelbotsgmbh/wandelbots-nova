"""
NATS message utilities.
"""


class Message:
    """A NATS message container."""

    def __init__(self, subject: str, data: bytes):
        """
        Initialize a NATS message.

        Args:
            subject: The NATS subject for the message
            data: The message data as bytes
        """
        self.subject = subject
        self.data = data
