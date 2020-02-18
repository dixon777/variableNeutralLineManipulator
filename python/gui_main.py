from variable_neutral_line_manipulator.display.window import App
from variable_neutral_line_manipulator.common import Logger

def main():
    App.run()
    
if __name__ == "__main__":
    import logging
    Logger.get().setLevel(logging.DEBUG)
    main()