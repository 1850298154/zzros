from time_utils import TimeManager, Calendar
from timer import Timer
import time
# Time management module usage
time_manager = TimeManager()
current_time = time_manager.get_current_time()
print("Current time:", time_manager.format_time(current_time))

# Timer module usage
def task():
    print("Executing task...")

timer = Timer()
timer.add_timer(task, 5)  # Execute task every 5 seconds
time.sleep(15)  # Wait for timers to execute
