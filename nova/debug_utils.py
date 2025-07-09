"""Debug utilities for inspecting Nova's behavior."""

import logging

import wandelbots_api_client as wb

from nova.core.logging import logger as nova_logger


def setup_extra_debug_logging():
    """Set up extra debug logging for the movement controller."""
    nova_logger.setLevel(logging.DEBUG)

    # Hook the PlaybackSpeedRequest to log when it's created
    original_request_init = wb.models.PlaybackSpeedRequest.__init__

    def playback_speed_request_init_hook(self, *args, **kwargs):
        original_request_init(self, *args, **kwargs)
        nova_logger.debug(f"PlaybackSpeedRequest created with {self.playback_speed_in_percent}%")

    wb.models.PlaybackSpeedRequest.__init__ = playback_speed_request_init_hook

    # Hook the PlaybackSpeedResponse to log when it's received
    if hasattr(wb.models, "PlaybackSpeedResponse"):
        try:
            original_response_init = wb.models.PlaybackSpeedResponse.__init__

            def playback_speed_response_init_hook(self, *args, **kwargs):
                original_response_init(self, *args, **kwargs)
                nova_logger.debug(
                    f"PlaybackSpeedResponse received with {getattr(self, 'playback_speed_response', 'unknown')}%"
                )

            wb.models.PlaybackSpeedResponse.__init__ = playback_speed_response_init_hook
        except Exception as e:
            nova_logger.debug(f"Error setting up PlaybackSpeedResponse hook: {e}")

    # Enhance the movement controller debugging
    try:
        from nova.core.movement_controller import move_forward

        def debug_movement_controller_wrapper(context):
            nova_logger.debug(
                f"*** move_forward called with effective_speed={context.effective_speed}, method_speed={context.method_speed}"
            )
            original_controller = move_forward(context)

            async def wrapped_controller(response_stream):
                nova_logger.debug("*** Starting debug movement controller")

                async for response in original_controller(response_stream):
                    nova_logger.debug(f"*** Yielding request type: {type(response).__name__}")
                    yield response

                nova_logger.debug("*** Finished debug movement controller")

            return wrapped_controller

        # Patch the movement controller
        import nova.core.movement_controller

        nova.core.movement_controller.move_forward = debug_movement_controller_wrapper
        nova_logger.debug("*** Patched move_forward function with debugging")

    except Exception as e:
        nova_logger.debug(f"Error setting up movement controller debugging: {e}")

    nova_logger.debug("Extra debug logging set up")
    return True
