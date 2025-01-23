from solutions.vortex_solution import MyDroneVortex


class MyDroneEval(MyDroneVortex):
    def __init__(self, **kwargs):
        super().__init__(signature="module manager", **kwargs)
