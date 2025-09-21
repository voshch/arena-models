
class BoundingBox(tuple[tuple[float, float], tuple[float, float], tuple[float, float]]):
    @property
    def min_x(self) -> float:
        return self[0][0]

    @property
    def max_x(self) -> float:
        return self[0][1]

    @property
    def min_y(self) -> float:
        return self[1][0]

    @property
    def max_y(self) -> float:
        return self[1][1]

    @property
    def min_z(self) -> float:
        return self[2][0]

    @property
    def max_z(self) -> float:
        return self[2][1]

    @classmethod
    def empty(cls) -> "BoundingBox":
        return cls(((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)))
