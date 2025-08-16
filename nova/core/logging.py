import nova.logger as nova_logger

# for backward compatibility, remove them in a appropriate release
LOG_LEVEL = nova_logger._LOG_LEVEL
LOG_FORMAT = nova_logger._LOG_FORMAT
LOG_DATETIME_FORMAT = nova_logger._LOG_DATETIME_FORMAT
LOGGER_NAME = nova_logger._LOGGER_NAME

formatter = nova_logger._formatter
handler = nova_logger._handler
logger = nova_logger.logger
