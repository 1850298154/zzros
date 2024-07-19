

from collections    import deque

from src            import tracker





class Random_Access_Queue():
    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def clear(self) -> None:
        # self. queue = deque(['A', 'B', 'C', 'D'])
        self. queue = deque( [] )
        self. done_queue = deque( [] )
        pass

    def del_index(self, index) -> None:
        # queue = deque(['A', 'B', 'C', 'D'])
        # 将deque转换为列表
        queue_list = list(self.queue)
        # 使用下标访问元素
        # print(queue_list[2])  # 输出：C
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(queue_list[index])  # 输出：C
            tracker.debug_tracker.instance.current_position_print()
        # 使用下标删除元素
        # del queue_list[1]
        del queue_list[index]
        # 将修改后的列表转换回deque
        self.queue = deque(queue_list)
        if self.debug_flag == True:
            # 打印修改后的deque
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque(['A', 'C', 'D'])
            tracker.debug_tracker.instance.current_position_print()
        pass

    def insert_index(self, index, ele) -> None:
        # self.queue = deque([1, 2, 3, 4, 5])
        # 在索引为2的位置插入元素
        # self.queue.insert(2, 6)
        self.queue.insert(index, ele)
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([1, 2, 6, 3, 4, 5])
            tracker.debug_tracker.instance.current_position_print()
        pass

    def remove_index(self, index) -> None:
        # 删除索引为3的元素
        # self.queue.remove(3)
        self.queue.remove(index)
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([1, 2, 6, 4, 5])
            tracker.debug_tracker.instance.current_position_print()
        pass

    def append(self, ele) -> None:
        # self.queue = deque([1, 2, 3])
        # self.queue.append(4)  # 在右端插入元素
        self.queue.append(ele)  # 在右端插入元素
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([1, 2, 3, 4])
            tracker.debug_tracker.instance.current_position_print()
        pass

    def ___appendleft(self, ele) -> None:
        tracker.debug_tracker.instance.current_position_print()
        tracker.violation_detector.instance.raise_violation_error(
            "Inserting an element at the left end"
        )
        # self.queue.appendleft(0)  # 在左端插入元素
        self.queue.appendleft(ele)  # 在左端插入元素
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([0, 1, 2, 3, 4])
            tracker.debug_tracker.instance.current_position_print()
        pass

    def ___pop(self) -> None:
        tracker.debug_tracker.instance.current_position_print()
        tracker.violation_detector.instance.raise_violation_error(
            "Delete element from right end"
        )
        ret = self.queue.pop()  # 从右端删除元素
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([0, 1, 2, 3])
            tracker.debug_tracker.instance.current_position_print()
        return ret
        pass

    def popleft(self) -> None:
        ele = self.queue.popleft()  # 从左端删除元素
        self.done_queue.append(ele)
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([1, 2, 3])
            tracker.debug_tracker.instance.pf(ele)  # 输出：deque([1, 2, 3])
            tracker.debug_tracker.instance.pf('self.done_queue')  # 输出：deque([1, 2, 3])
            tracker.debug_tracker.instance.pf(self.done_queue)  # 输出：deque([1, 2, 3])
            tracker.debug_tracker.instance.current_position_print()
        return ele
        pass

    def len(self) -> None:
        size = len(self.queue)
        return size
        pass

    def rotate(self, offset_right) -> None:
        # self.queue.rotate(1)  # 右旋转1步
        # print(self.queue)  # 输出：deque([3, 1, 2])
        # self.queue.rotate(-1)  # 左旋转1步
        # print(self.queue)  # 输出：deque([1, 2, 3])
        self.queue.rotate(offset_right)  # 右旋转1步
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(self.queue)  # 输出：deque([3, 1, 2])
            tracker.debug_tracker.instance.current_position_print()
        pass

    def first_element(self) -> None:
        first_element = self.queue[0]  # 访问第一个元素
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf(first_element)  
            tracker.debug_tracker.instance.current_position_print()
        return first_element
        pass

    def print(self) -> None:
        tracker.debug_tracker.instance.pf(self.queue)  
        tracker.debug_tracker.instance.current_position_print()    


instance = Random_Access_Queue()

