
import math


class Load_Balancer:
    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.mount_debug_flag = True
        self.config_debug_flag = True
        pass

    def start(self) -> None:
        pass

    def clear(self) -> None:
        pass

    def mount_input(self, input_list):
        if self.mount_debug_flag == True:
            from src    import tracker 
            tracker.debug_tracker.instance.pf('input_list')
            tracker.debug_tracker.instance.pf(input_list) 
            tracker.debug_tracker.instance.current_position_print()
        pass

    def assign_drones(self, n_drone, m_region, region_sizes):
        """
        The `assign_drones` function takes in the number of drones `n`, number of regions `m`, and sizes of
        regions, and assigns drones to each region based on size while ensuring exactly `n` drones are
        assigned.
        
        :param n:               number of drones `n`
        :param m:               number of regions `m`
        :param region_sizes:    sizes of regions, region_sizes = [20, 15, 30, 10, 25]

        :return: The `assign_drones` function returns a list that represents the number of drones assigned
        to each region.
        """
        
        if all(x <= 0 for x in region_sizes):
            from src    import tracker 
            tracker.debug_tracker.instance.pf('region_sizes all <= 0') 
            tracker.debug_tracker.instance.pf(region_sizes) 
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: region_sizes all <= 0")
        if n_drone <= 0 or m_region <= 0 or n_drone < m_region:
            from src    import tracker 
            tracker.debug_tracker.instance.pf('n, m, region_sizes       : input is') 
            tracker.debug_tracker.instance.pf(n_drone) 
            tracker.debug_tracker.instance.pf(m_region) 
            tracker.debug_tracker.instance.pf(region_sizes) 
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: n must be greater than m and both must be positive integers.")

        # Initialize list to hold number of drones assigned to each region
        drones_assigned = [0] * m_region
        # print('drones_assigned')      # drones_assigned
        # print(drones_assigned)        # [0, 0, 0, 0, 0]

        # Calculate total size of all regions
        total_size = sum(region_sizes)

        # Calculate the percentage of area per region
        percentage_of_area = [ region_sizes[i] / total_size 
                                for i in range(m_region)]
        floor_assigned =     [  math.floor( n_drone * percentage_of_area[i])  
                                for i in range(m_region)
                                ]
        ceil_assigned  =     [  math.ceil(  n_drone * percentage_of_area[i])   
                                for i in range(m_region)
                                ]
        round_assigned =     [  round(      n_drone * percentage_of_area[i])   
                                for i in range(m_region)
                                ]
        # Calculate how many drones should be assigned to each region based on size
        for i in range(m_region):
            drones_assigned[i] = int(round(n_drone * (region_sizes[i] / total_size)))
            # drones_assigned[i] = int(math.ceil(n_drone * (region_sizes[i] / total_size)))

        # Adjust drone assignments to ensure exactly n drones are assigned
        while sum(drones_assigned) != n_drone:
            if sum(drones_assigned) < n_drone:
                # Assign an extra drone to the largest region
                # max_index = drones_assigned.index(max(drones_assigned))
                max_index = max(
                                [
                                    i 
                                    for i in range(m_region) 
                                    if drones_assigned[i] > 0
                                ], 
                                key=lambda x: drones_assigned[x])
                drones_assigned[max_index] += 1
            elif sum(drones_assigned) > n_drone:
                # Remove a drone from the smallest region that has more than 0 drones
                # min_index = min([i for i in range(m_region) if drones_assigned[i] > 0], key=lambda x: drones_assigned[x])
                min_index = min(
                                [
                                    i 
                                    for i in range(m_region) 
                                    if drones_assigned[i] > 0
                                ], 
                                key=lambda x: drones_assigned[x])
                drones_assigned[min_index] -= 1

        return drones_assigned

"""
# Example usage:
n = 10  # Total number of drones
m = 5   # Number of regions

# Example sizes of regions (could be any numeric values, for simplicity they sum up to 100)
region_sizes = [20, 15, 30, 10, 25]

# Assign drones
try:
    assigned_drones = assign_drones(n, m, region_sizes)
    print("Assigned drones per region:", assigned_drones)
except ValueError as e:
    print(e)
------------------------------------------------

drones_assigned
[0, 0, 0, 0, 0]
Assigned drones per region: [2, 2, 3, 1, 2]

------------------------------------------------

        """
if __name__ == '__main__':
    # Example usage:
    n = 10  # Total number of drones
    m = 5   # Number of regions

    # Example sizes of regions (could be any numeric values, for simplicity they sum up to 100)
    region_sizes = [20, 15, 30, 10, 25]

    
    # Example usage:
    n = 5   # Total number of drones
    m = 5   # Number of regions

    # Example sizes of regions (could be any numeric values, for simplicity they sum up to 100)
    region_sizes = [20, 15, 30, 10, 10000]

    # Assign drones
    try:
        assigned_drones = Load_Balancer().assign_drones(n, m, region_sizes)
        print("Assigned drones per region:", assigned_drones)  # Assigned drones per region: [2, 2, 2, 1, 3]
    except ValueError as e:
        print(e)