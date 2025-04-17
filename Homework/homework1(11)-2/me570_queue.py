"""
A pedagogical implementation of a priority queue
"""

from numbers import Number


class PriorityQueue:
    """ Implements a priority queue """

    def __init__(self):
        """
        Initializes the internal attribute  queue to be an empty list.
        """
        self.queue_list = []

    def check(self):
        """
        Check that the internal representation is a list of (key,value) pairs,
        where value is numerical
        """
        is_valid = True
        for pair in self.queue_list:
            if len(pair) != 2:
                is_valid = False
                break
            if not isinstance(pair[1], Number):
                is_valid = False
                break
        return is_valid

    def insert(self, key, cost):
        """
        Add an element to the queue.
        """
        self.queue_list.append((key, cost))

    def min_extract(self):
        """
        Extract the element with minimum cost from the queue.
        """
        if len(self.queue_list) == 0:
            cost_best = None
            key_best = None
        else:
            idx_best = 0
            cost_best = self.queue_list[0][1]

            for idx in range(1, len(self.queue_list)):
                cost = self.queue_list[idx][1]
                if cost < cost_best:
                    cost_best = cost
                    idx_best = idx

            key_best = self.queue_list[idx_best][0]
            del self.queue_list[idx_best]
        return key_best, cost_best

    def is_member(self, key):
        """
        Check whether an element with a given key is in the queue or not.
        """
        bTrue = False
        if any(pair[0] == key for pair in self.queue_list):
            bTrue = True
        return bTrue


def priority_test():
    """
    The function should perform the following steps. 
    Make sure to print the contents of the queue after each step.
    1) Initialize an empty queue as the object p_queue .
    2) Add three elements (as shown in Table 2 and in that order) to that queue.
    3) Extract a minimum element. Print the key and cost of such element.
    4) Add another element (as shown in Table 2).
    5) Check if the following keys are present: ’Apples’ , ’Bananas’ , ’(1,5)’ . 
    Print the result after each check.
    6) Remove all elements by repeated extractions. Print the extracted key and 
    cost after each extraction.
    """
    p_queue = PriorityQueue()
    p_queue.check()
    print(p_queue.queue_list)

    key, cost = p_queue.min_extract()
    print(key, cost)
    print(p_queue.queue_list)

    p_queue.insert('Oranges', 4.5)
    p_queue.insert('Apples', 1)
    p_queue.insert('Bananas', 2.7)
    print(p_queue.queue_list)

    key, cost = p_queue.min_extract()
    print(key, cost)
    print(p_queue.queue_list)

    p_queue.insert('Cantaloupe', 3)
    print(p_queue.queue_list)

    print(p_queue.is_member("Apples"))
    print(p_queue.is_member("Bananas"))
    print(p_queue.is_member('(1,5)'))

    while p_queue.queue_list:
        key, cost = p_queue.min_extract()
        print(key, cost)
        print(p_queue.queue_list)
