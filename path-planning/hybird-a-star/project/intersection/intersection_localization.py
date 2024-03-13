"""
Intersection Localization Module
"""


class IntersectionLocalization:
    @staticmethod
    def is_in_intersection(vehicle):
        return -4 < vehicle[1] <= 4 and -4 < vehicle[2] <= 4

    @staticmethod
    def is_in_up_source(vehicle):
        return 0 < vehicle[1] <= 4 and vehicle[2] > 0

    @staticmethod
    def is_in_up_sink(vehicle):
        return 0 < vehicle[1] <= 4 and vehicle[2] <= 0

    @staticmethod
    def is_in_right_source(vehicle):
        return 0 < vehicle[2] <= 4 and vehicle[1] <= 4.5

    @staticmethod
    def is_in_right_sink(vehicle):
        return 0 < vehicle[2] <= 4 and vehicle[1] > 4.5

    @staticmethod
    def is_in_left_source(vehicle):
        return -4 <= vehicle[2] <= 0 and vehicle[1] > 0.5

    @staticmethod
    def is_in_left_sink(vehicle):
        return -4 <= vehicle[2] <= 0 and vehicle[1] <= 0.5
