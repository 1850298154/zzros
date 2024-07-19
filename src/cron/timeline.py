import datetime

class TimelineEvent:
    def __init__(self, title, date, description=""):
        self.title = title
        self.date = date
        self.description = description
    
    def __str__(self):
        return f"{self.date}: {self.title}\n    {self.description}"

class Timeline:
    def __init__(self):
        self.initialize()
        pass

    def initialize(self) -> None:
        self.events = []
        self.sys_boot_time                  = datetime.datetime.now()
        self.sys_boot_time_str              = self.sys_boot_time.strftime("%Y%m%d-%H%M%S")
        self.sys_boot_time_str_filename     = self.sys_boot_time_str + '-stdout.sh' 
        self.directory = r'zmemorandum/udp/'
        self.sys_boot_time_str_filename = self.directory + self.sys_boot_time_str_filename
        pass

    def start(self) -> None:
        pass

    def add_event(self, event):
        self.events.append(event)
    
    def sort_events_by_date(self):
        self.events.sort(key=lambda x: x.date)
    
    def show_timeline(self):
        if not self.events:
            print("No events in the timeline.")
            return
        
        for event in self.events:
            print(event)
    
    def find_event_by_title(self, title):
        for event in self.events:
            if event.title.lower() == title.lower():
                return event.date
        return None

instance = Timeline()

"""
# Example usage:
if __name__ == "__main__":
    # Create a timeline
    timeline = Timeline()

    # Adding events to the timeline
    event1 = TimelineEvent("First Event", datetime.datetime(2024, 6, 25), "This is the first event.")
    event2 = TimelineEvent("Second Event", datetime.datetime(2024, 6, 26), "This is the second event.")
    event3 = TimelineEvent("Third Event", datetime.datetime(2024, 6, 24), "This is the third event.")

    timeline.add_event(event1)
    timeline.add_event(event2)
    timeline.add_event(event3)

    # Sort events by date
    timeline.sort_events_by_date()

    # Show the timeline
    timeline.show_timeline()

    # Find an event by title
    search_title = "Second Event"
    event_date = timeline.find_event_by_title(search_title)
    if event_date:
        print(f"The event '{search_title}' occurred on: {event_date}")
    else:
        print(f"Event '{search_title}' not found in the timeline.")

"""