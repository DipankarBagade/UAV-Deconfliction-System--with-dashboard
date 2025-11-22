# backend/core/spatial_index.py
"""
Spatial indexing system for efficient conflict detection
Uses Grid-based partitioning + KD-Tree (FlytBase approach)
"""

import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple, Dict, Set, Optional
from dataclasses import dataclass, field
import time

# Try relative import first, fall back to absolute
try:
    from .models import Mission, Trajectory
except ImportError:
    from models import Mission, Trajectory


@dataclass
class GridCell:
    """Represents a 3D grid cell in the airspace"""
    cell_id: Tuple[int, int, int]  # (i, j, k) indices
    bounds: Tuple[np.ndarray, np.ndarray]  # (min_coords, max_coords)
    drone_ids: Set[str] = field(default_factory=set)
    
    def contains_point(self, point: np.ndarray) -> bool:
        """Check if a 3D point is inside this cell"""
        min_coords, max_coords = self.bounds
        return np.all(point >= min_coords) and np.all(point < max_coords)


class SpatialGrid:
    """
    3D Grid-based spatial partitioning
    Stage 1 of FlytBase's two-stage conflict detection
    """
    
    def __init__(self, 
                 airspace_bounds: Tuple[np.ndarray, np.ndarray],
                 cell_size: float = 100.0):
        """
        Initialize spatial grid
        
        Args:
            airspace_bounds: ((min_x, min_y, min_z), (max_x, max_y, max_z))
            cell_size: Size of each grid cell in meters
        """
        self.min_bounds, self.max_bounds = airspace_bounds
        self.cell_size = cell_size
        
        # Calculate grid dimensions
        self.grid_dims = np.ceil((self.max_bounds - self.min_bounds) / cell_size).astype(int)
        
        # Dictionary to store cells: cell_id -> GridCell
        self.cells: Dict[Tuple[int, int, int], GridCell] = {}
        
        # Dictionary to store drone trajectories: drone_id -> trajectory points
        self.drone_trajectories: Dict[str, np.ndarray] = {}
        
        print(f"✅ Spatial Grid initialized: {self.grid_dims} cells "
              f"({self.grid_dims[0]}x{self.grid_dims[1]}x{self.grid_dims[2]})")
    
    def _get_cell_id(self, point: np.ndarray) -> Tuple[int, int, int]:
        """
        Get cell ID for a 3D point
        
        Args:
            point: [x, y, z] coordinates
        
        Returns:
            (i, j, k) cell indices
        """
        # Clamp point to airspace bounds
        point = np.clip(point, self.min_bounds, self.max_bounds - 0.001)
        
        # Calculate cell indices
        indices = ((point - self.min_bounds) / self.cell_size).astype(int)
        
        # Ensure within grid dimensions
        indices = np.clip(indices, 0, self.grid_dims - 1)
        
        return tuple(indices)
    
    def _get_cell_bounds(self, cell_id: Tuple[int, int, int]) -> Tuple[np.ndarray, np.ndarray]:
        """Get the 3D bounding box for a cell"""
        i, j, k = cell_id
        
        min_coords = self.min_bounds + np.array([i, j, k]) * self.cell_size
        max_coords = min_coords + self.cell_size
        
        # Clamp to airspace bounds
        max_coords = np.minimum(max_coords, self.max_bounds)
        
        return min_coords, max_coords
    
    def _ensure_cell_exists(self, cell_id: Tuple[int, int, int]) -> GridCell:
        """Create cell if it doesn't exist"""
        if cell_id not in self.cells:
            bounds = self._get_cell_bounds(cell_id)
            self.cells[cell_id] = GridCell(cell_id=cell_id, bounds=bounds)
        
        return self.cells[cell_id]
    
    def index_trajectory(self, drone_id: str, trajectory: Trajectory):
        """
        Index a drone's trajectory in the grid
        
        Args:
            drone_id: Unique drone identifier
            trajectory: Trajectory object with sampled points
        """
        # Store trajectory points
        self.drone_trajectories[drone_id] = trajectory.points
        
        # Index each point in the trajectory
        for point in trajectory.points:
            # Get 3D position (ignore time for spatial indexing)
            position = point[:3]
            
            # Get cell ID
            cell_id = self._get_cell_id(position)
            
            # Add drone to cell
            cell = self._ensure_cell_exists(cell_id)
            cell.drone_ids.add(drone_id)
    
    def get_nearby_drones(self, 
                         point: np.ndarray, 
                         search_radius: float) -> Set[str]:
        """
        Get all drones within search radius of a point
        Uses grid cells for fast broad-phase filtering
        
        Args:
            point: [x, y, z] position
            search_radius: Search distance in meters
        
        Returns:
            Set of drone IDs that might be nearby
        """
        # Calculate how many cells to search in each direction
        cells_to_check = int(np.ceil(search_radius / self.cell_size))
        
        # Get the cell containing the point
        center_cell_id = self._get_cell_id(point)
        
        # Collect all nearby drones
        nearby_drones = set()
        
        # Check neighboring cells
        for di in range(-cells_to_check, cells_to_check + 1):
            for dj in range(-cells_to_check, cells_to_check + 1):
                for dk in range(-cells_to_check, cells_to_check + 1):
                    # Calculate neighbor cell ID
                    neighbor_id = (
                        center_cell_id[0] + di,
                        center_cell_id[1] + dj,
                        center_cell_id[2] + dk
                    )
                    
                    # Check if cell is within grid bounds
                    if (0 <= neighbor_id[0] < self.grid_dims[0] and
                        0 <= neighbor_id[1] < self.grid_dims[1] and
                        0 <= neighbor_id[2] < self.grid_dims[2]):
                        
                        # Get drones in this cell
                        if neighbor_id in self.cells:
                            nearby_drones.update(self.cells[neighbor_id].drone_ids)
        
        return nearby_drones
    
    def get_statistics(self) -> Dict:
        """Get statistics about the spatial grid"""
        total_cells = len(self.cells)
        occupied_cells = sum(1 for cell in self.cells.values() if len(cell.drone_ids) > 0)
        total_drones = len(self.drone_trajectories)
        
        # Calculate average drones per cell
        drones_per_cell = [len(cell.drone_ids) for cell in self.cells.values()]
        avg_drones_per_cell = np.mean(drones_per_cell) if drones_per_cell else 0
        max_drones_per_cell = max(drones_per_cell) if drones_per_cell else 0
        
        return {
            "total_cells": total_cells,
            "occupied_cells": occupied_cells,
            "total_drones": total_drones,
            "avg_drones_per_cell": avg_drones_per_cell,
            "max_drones_per_cell": max_drones_per_cell
        }


class KDTreeIndex:
    """
    KD-Tree based spatial index for precise distance queries
    Stage 2 of FlytBase's two-stage conflict detection
    """
    
    def __init__(self, leaf_size: int = 40):
        """
        Initialize KD-Tree index
        
        Args:
            leaf_size: Number of points at which to switch to brute-force
        """
        self.leaf_size = leaf_size
        self.trees: Dict[str, cKDTree] = {}  # drone_id -> KDTree
        self.points: Dict[str, np.ndarray] = {}  # drone_id -> points array
    
    def index_trajectory(self, drone_id: str, trajectory: Trajectory):
        """
        Build KD-Tree for a drone's trajectory
        
        Args:
            drone_id: Unique drone identifier
            trajectory: Trajectory object
        """
        # Extract 3D positions (ignore time for KD-Tree)
        points_3d = trajectory.points[:, :3]
        
        # Store points
        self.points[drone_id] = trajectory.points  # Keep full 4D for temporal checks
        
        # Build KD-Tree
        self.trees[drone_id] = cKDTree(points_3d, leafsize=self.leaf_size)
    
    def query_radius(self, 
                    drone_id: str, 
                    point: np.ndarray, 
                    radius: float) -> List[int]:
        """
        Find all points within radius of a query point
        
        Args:
            drone_id: Drone to query
            point: [x, y, z] query position
            radius: Search radius in meters
        
        Returns:
            List of point indices within radius
        """
        if drone_id not in self.trees:
            return []
        
        tree = self.trees[drone_id]
        indices = tree.query_ball_point(point, radius)
        
        return indices
    
    def nearest_neighbors(self,
                         drone_id: str,
                         point: np.ndarray,
                         k: int = 1) -> Tuple[np.ndarray, np.ndarray]:
        """
        Find k nearest neighbors to a query point
        
        Args:
            drone_id: Drone to query
            point: [x, y, z] query position
            k: Number of neighbors
        
        Returns:
            (distances, indices) arrays
        """
        if drone_id not in self.trees:
            return np.array([]), np.array([])
        
        tree = self.trees[drone_id]
        distances, indices = tree.query(point, k=k)
        
        return distances, indices


class HybridSpatialIndex:
    """
    Hybrid spatial index combining Grid + KD-Tree
    This is the FlytBase approach for scalable conflict detection
    """
    
    def __init__(self,
                 airspace_bounds: Tuple[Tuple[float, float, float], 
                                       Tuple[float, float, float]],
                 cell_size: float = 100.0,
                 leaf_size: int = 40):
        """
        Initialize hybrid spatial index
        
        Args:
            airspace_bounds: ((min_x, min_y, min_z), (max_x, max_y, max_z))
            cell_size: Grid cell size in meters
            leaf_size: KD-Tree leaf size
        """
        # Convert to numpy arrays
        min_bounds = np.array(airspace_bounds[0])
        max_bounds = np.array(airspace_bounds[1])
        
        # Initialize both indexing structures
        self.grid = SpatialGrid((min_bounds, max_bounds), cell_size)
        self.kdtree = KDTreeIndex(leaf_size)
        
        # Track indexed drones
        self.indexed_drones: Set[str] = set()
        
        print(f"✅ Hybrid Spatial Index initialized")
        print(f"   Grid: {cell_size}m cells")
        print(f"   KD-Tree: leaf_size={leaf_size}")
    
    def index_mission(self, mission: Mission, trajectory: Trajectory):
        """
        Index a mission's trajectory in both structures
        
        Args:
            mission: Mission object
            trajectory: Trajectory object
        """
        drone_id = mission.drone_id
        
        # Index in grid (for broad phase)
        self.grid.index_trajectory(drone_id, trajectory)
        
        # Index in KD-Tree (for narrow phase)
        self.kdtree.index_trajectory(drone_id, trajectory)
        
        # Mark as indexed
        self.indexed_drones.add(drone_id)
    
    def find_potential_conflicts(self,
                                point: np.ndarray,
                                search_radius: float,
                                exclude_drones: Optional[Set[str]] = None) -> Set[str]:
        """
        Stage 1: Use grid to find drones that might be in conflict
        
        Args:
            point: [x, y, z] position to check
            search_radius: Safety buffer radius
            exclude_drones: Set of drone IDs to exclude from results
        
        Returns:
            Set of drone IDs that might be in conflict
        """
        exclude_drones = exclude_drones or set()
        
        # Use grid for fast broad-phase filtering
        nearby_drones = self.grid.get_nearby_drones(point, search_radius)
        
        # Remove excluded drones
        nearby_drones -= exclude_drones
        
        return nearby_drones
    
    def check_precise_distance(self,
                              drone_id: str,
                              point: np.ndarray,
                              max_distance: float) -> Tuple[float, Optional[int]]:
        """
        Stage 2: Use KD-Tree for precise distance calculation
        
        Args:
            drone_id: Drone to check against
            point: [x, y, z] position
            max_distance: Maximum distance to consider
        
        Returns:
            (minimum_distance, closest_point_index)
        """
        # Find nearest neighbor
        distances, indices = self.kdtree.nearest_neighbors(drone_id, point, k=1)
        
        # Handle empty results - check if it's an array first
        if isinstance(distances, np.ndarray):
            if distances.size == 0:
                return float('inf'), None
            min_distance = float(distances[0])
            closest_idx = int(indices[0])
        else:
            # It's already a scalar
            if not distances:  # Check if empty/None
                return float('inf'), None
            min_distance = float(distances)
            closest_idx = int(indices)
        
        return min_distance, closest_idx
    
    def get_statistics(self) -> Dict:
        """Get comprehensive statistics"""
        grid_stats = self.grid.get_statistics()
        
        return {
            "indexed_drones": len(self.indexed_drones),
            "grid_stats": grid_stats,
            "kdtree_count": len(self.kdtree.trees)
        }
    
    def clear(self):
        """Clear all indexed data"""
        self.grid = SpatialGrid(
            (self.grid.min_bounds, self.grid.max_bounds),
            self.grid.cell_size
        )
        self.kdtree = KDTreeIndex(self.kdtree.leaf_size)
        self.indexed_drones.clear()


if __name__ == "__main__":
    print("Testing Spatial Index System...")
    
    # Use absolute import for testing
    import sys
    from pathlib import Path
    
    # Add backend to path
    backend_path = Path(__file__).parent.parent
    sys.path.insert(0, str(backend_path))
    
    from core.models import create_mission_from_waypoints, Trajectory
    
    # Define airspace
    airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
    
    # Create hybrid index
    index = HybridSpatialIndex(
        airspace_bounds=airspace_bounds,
        cell_size=100.0
    )
    
    print("\n1. Creating test missions...")
    
    # Create several test missions
    missions = []
    for i in range(10):
        waypoints = [
            (i * 100, i * 100, 50, 0),
            (i * 100 + 500, i * 100 + 500, 100, 30),
            (i * 100 + 1000, i * 100 + 200, 75, 60)
        ]
        
        mission = create_mission_from_waypoints(
            mission_id=f"TEST_{i:03d}",
            waypoint_coords=waypoints,
            drone_id=f"UAV_{i:04d}"
        )
        missions.append(mission)
    
    print(f"✅ Created {len(missions)} test missions")
    
    print("\n2. Indexing trajectories...")
    start_time = time.time()
    
    for mission in missions:
        trajectory = Trajectory(mission=mission, sampling_rate=1.0)
        index.index_mission(mission, trajectory)
    
    index_time = time.time() - start_time
    print(f"✅ Indexed {len(missions)} trajectories in {index_time:.4f}s")
    
    print("\n3. Testing spatial queries...")
    
    # Test point
    test_point = np.array([500, 500, 75])
    search_radius = 50.0
    
    # Stage 1: Grid-based broad phase
    start_time = time.time()
    nearby_drones = index.find_potential_conflicts(test_point, search_radius)
    broad_phase_time = time.time() - start_time
    
    print(f"✅ Broad phase found {len(nearby_drones)} potential conflicts in {broad_phase_time*1000:.2f}ms")
    
    # Stage 2: KD-Tree narrow phase
    start_time = time.time()
    for drone_id in nearby_drones:
        distance, idx = index.check_precise_distance(drone_id, test_point, search_radius)
        if distance < search_radius:
            print(f"   Conflict with {drone_id}: distance={distance:.2f}m")
    
    narrow_phase_time = time.time() - start_time
    print(f"✅ Narrow phase completed in {narrow_phase_time*1000:.2f}ms")
    
    print("\n4. Index statistics:")
    stats = index.get_statistics()
    print(f"   Indexed drones: {stats['indexed_drones']}")
    print(f"   Grid cells: {stats['grid_stats']['occupied_cells']}/{stats['grid_stats']['total_cells']}")
    print(f"   Avg drones per cell: {stats['grid_stats']['avg_drones_per_cell']:.2f}")
    
    print("\n🚀 Spatial index system working correctly!")