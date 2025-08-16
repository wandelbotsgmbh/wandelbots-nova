import nova.logger as nova_logger

# for backward compatibility, remove them in a appropriate release
LOG_LEVEL = nova_logger.LOG_LEVEL
LOG_FORMAT = nova_logger.LOG_FORMAT
LOG_DATETIME_FORMAT = nova_logger.LOG_DATETIME_FORMAT
LOGGER_NAME = nova_logger.LOGGER_NAME

formatter = nova_logger.formatter
handler = nova_logger.handler
logger = nova_logger.logger
