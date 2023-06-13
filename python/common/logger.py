import logging
import inspect


## private class, forbiden to import.
class __SingletonLogger:
    """Log Module."""
    _instance = None
    def __new__(cls, name):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.logger = logging.getLogger(name)
            cls._instance.logger.setLevel(logging.DEBUG)
            formatter = logging.Formatter('[%(levelname)s-%(asctime)s][%(pathname)s][line:%(lineno)d]: %(message)s')
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.DEBUG)
            console_handler.setFormatter(formatter)
            cls._instance.logger.addHandler(console_handler)
        return cls._instance

    def __init__(self, name) -> None:
        self.debug = self.logger.debug
        self.info = self.logger.info
        self.warning = self.logger.info
        self.error = self.logger.error
        self.critical = self.logger.critical
        self.fatal = self.logger.fatal

    def find_caller(self, stack_info=False, stacklevel=1):
        f = inspect.currentframe()
        if f is not None:
            f = f.f_back
        orig_f = f
        while f and stacklevel > 1:
            f = f.f_back
            stacklevel -= 1
        if not f:
            f = orig_f
        co = f.f_code
        funcName = co.co_name
        filename = co.co_filename
        lineno = f.f_lineno
        return (filename, lineno, funcName, None)

    def isEnabledFor(self, level):
        return self.logger.isEnabledFor(level)
    

# only can import logger
logger = __SingletonLogger(name='navigation')

if __name__ == '__main__':
    logger.fatal("welecome to use log module.")