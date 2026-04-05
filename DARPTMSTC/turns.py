class turns:
    def __init__(self, paths):
        """
        paths: List of paths, where each path is a list of (x, y) tuples.
        """
        self.paths = paths
        self.turns = []
        self.avg = 0
        self.std = 0

    def count_turns(self):
        """
        Count the number of turns for each path and store in self.turns.
        """
        self.turns = [self._count_turns_single(path) for path in self.paths]

    def _count_turns_single(self, path):
        """
        Count the number of turns in a single path.
        A turn is counted when the direction vector changes.
        """
        if len(path) < 3:
            return 0
        turns = 0
        for i in range(2, len(path)):
            # Extract first two elements of each point
            x0, y0 = path[i-2][0], path[i-2][1]
            x1, y1 = path[i-1][0], path[i-1][1]
            x2, y2 = path[i][0], path[i][1]
            # Calculate direction vectors
            dx1 = x1 - x0
            dy1 = y1 - y0
            dx2 = x2 - x1
            dy2 = y2 - y1
            # Check if direction changed
            if dx1 * dy2 != dx2 * dy1:
                turns += 1
        return turns

    def find_avg_and_std(self):
        """
        Compute the average and standard deviation of the number of turns.
        """
        import numpy as np
        if not self.turns:
            self.avg = 0
            self.std = 0
        else:
            self.avg = float(np.mean(self.turns))
            self.std = float(np.std(self.turns))

    def __str__(self):
        return f"Turns: {self.turns}\nAverage: {self.avg:.3f}\nStandard Deviation: {self.std:.3f}\n"
