# src/exploration_navigation/waterfill_navigation.py

import numpy as np

class WaterfillNavigation:
    def __init__(self, map_size=(10, 10)):
        self.map_size = map_size
        self.map_grid = np.zeros(map_size)

    def update_discovered_area(self, discovered_area):
        self.map_grid = np.logical_or(self.map_grid, discovered_area)

    def calculate_scores(self):
        score_grid = np.ones(self.map_size) * np.inf
        score_grid[np.logical_not(self.map_grid)] = 0

        for i in range(self.map_size[0]):
            for j in range(self.map_size[1]):
                if score_grid[i, j] != 0:
                    for ni in range(max(0, i - 1), min(self.map_size[0], i + 2)):
                        for nj in range(max(0, j - 1), min(self.map_size[1], j + 2)):
                            if score_grid[ni, nj] < np.inf:
                                score_grid[i, j] = min(score_grid[i, j], score_grid[ni, nj] + 1)

        return score_grid

    def get_navigation_target(self):
        score_grid = self.calculate_scores()
        max_score_index = np.unravel_index(np.argmax(score_grid), score_grid.shape)
        return max_score_index
