"""WebSocket Control for Nova Programs

This module provides WebSocket-based external control for Nova robot programs.
When enabled via the @nova.program decorator, it allows external tools and
applications to control robot playback in real-time.

Usage:
    @nova.program(
        name="My Robot Program",
        external_control=nova.external_control.WebSocketControl()
    )
    async def my_program():
        # Your robot program here
        pass
"""

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from nova.external_control.websocket_control import NovaWebSocketServer

from nova.core.logging import logger


class WebSocketControl:
    """
    WebSocket control configuration for Nova programs.

    When added to a @nova.program decorator, this enables real-time external control
    via WebSocket connection on localhost:8765.

    Args:
        port: WebSocket server port (default: 8765)
        host: WebSocket server host (default: "localhost")
        auto_start: Whether to start server automatically (default: True)

    Example:
        @nova.program(
            name="Robot Demo",
            external_control=nova.external_control.WebSocketControl(port=8765)
        )
        async def robot_demo():
            # External clients can now control this program
            pass
    """

    def __init__(self, port: int = 8765, host: str = "localhost", auto_start: bool = True):
        self.port = port
        self.host = host
        self.auto_start = auto_start
        self._server: Optional["NovaWebSocketServer"] = None

    async def start(self) -> None:
        """Start the WebSocket control server."""
        if not self.auto_start:
            return

        try:
            from nova.core.playback_control import get_playback_manager
            from nova.external_control.websocket_control import start_websocket_server

            logger.info(f"Starting WebSocket control server on {self.host}:{self.port}")
            self._server = start_websocket_server(host=self.host, port=self.port)

            # Notify that a program with external control has started
            manager = get_playback_manager()
            manager.start_program(getattr(self, "_program_name", None))

            logger.info("✅ WebSocket control server started - external clients can now connect")

        except ImportError:
            logger.warning("WebSocket control not available - websockets library not installed")
        except Exception as e:
            logger.error(f"Failed to start WebSocket control server: {e}")

    async def stop(self) -> None:
        """Stop the WebSocket control server."""
        if self._server:
            try:
                from nova.core.playback_control import get_playback_manager
                from nova.external_control.websocket_control import stop_websocket_server

                # Notify that the program has stopped
                manager = get_playback_manager()
                manager.stop_program(getattr(self, "_program_name", None))

                stop_websocket_server()
                logger.info("WebSocket control server stopped")
            except Exception as e:
                logger.error(f"Error stopping WebSocket control server: {e}")
            finally:
                self._server = None

    def set_program_name(self, name: str) -> None:
        """Set the program name for event tracking"""
        self._program_name = name

    def __repr__(self) -> str:
        return (
            f"WebSocketControl(port={self.port}, host='{self.host}', auto_start={self.auto_start})"
        )
