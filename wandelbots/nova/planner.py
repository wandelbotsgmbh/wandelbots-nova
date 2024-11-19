from abc import ABC, abstractmethod

class Planner(ABC):

    @abstractmethod
    def plan(self):
        pass






class DefaultPlanner(Planner):
    pass