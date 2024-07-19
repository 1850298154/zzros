import threading
import time

class Timer:
    def __init__(self):
        self.timers = []

    def add_timer(self, task, interval):
        timer = threading.Timer(interval, task)
        self.timers.append(timer)
        timer.start()

    def remove_timer(self, task):
        for timer in self.timers:
            if timer.function == task:
                timer.cancel()
                self.timers.remove(timer)

    def pause_timer(self, task):
        for timer in self.timers:
            if timer.function == task:
                timer.cancel()

    def resume_timer(self, task):
        for timer in self.timers:
            if timer.function == task:
                new_timer = threading.Timer(timer.interval, timer.function)
                self.timers.append(new_timer)
                new_timer.start()
