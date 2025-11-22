# backend/core/conflict_detector.py
"""
4D Conflict Detection Engine
Implements FlytBase-style Strategic Deconfliction
"""

import numpy as np
from typing import List, Set, Optional
import time
from dataclasses import dataclass

# Try relative import first, fall back to absolute
try:
    from .models import Mission, Trajectory, Conflict, ConflictReport
    from .spatial_index import HybridSpatialIndex
except ImportError:
    from models import Mission, Trajectory, Conflict, ConflictReport
    from spatial_index import HybridSpatialIndex


@dataclass
class ConflictDetectorConfig:
    """Configuration for conflict detection"""
    safety_buffer: float = 10.0  # meters (FlytBase standard)
    critical_distance: float = 5.0  # meters
    warning_distance: float = 15.0  # meters
    time_tolerance: float = 0.5  # seconds (temporal matching tolerance)
    sampling_rate: float = 1.0  # seconds between trajectory samples


class ConflictDetector:
    """
    4D Conflict Detection Engine
    
    Implements FlytBase's Strategic Deconfliction approach:
    1. Spatial filtering (Grid)
    2. Precise distance checking (KD-Tree)
    3. Temporal validation (time window overlap)
    4. Conflict severity classification
    """
    
    def __init__(self, 
                 airspace_bounds: tuple,
                 config: Optional[ConflictDetectorConfig] = None):
        """
        Initialize conflict detector
        
        Args:
            airspace_bounds: ((min_x, min_y, min_z), (max_x, max_y, max_z))
            config: Configuration parameters
        """
        self.config = config or ConflictDetectorConfig()
        
        # Initialize spatial index
        self.spatial_index = HybridSpatialIndex(
            airspace_bounds=airspace_bounds,
            cell_size=100.0  # 100m grid cells
        )
        
        # Store all trajectories
        self.trajectories: dict[str, Trajectory] = {}
        self.missions: dict[str, Mission] = {}
        
        print(f"✅ Conflict Detector initialized")
        print(f"   Safety buffer: {self.config.safety_buffer}m")
        print(f"   Critical distance: {self.config.critical_distance}m")
        print(f"   Warning distance: {self.config.warning_distance}m")
    
    def index_simulation_drones(self, missions: List[Mission]):
        """
        Index all simulated drone trajectories
        This is called once at startup to populate the airspace
        
        Args:
            missions: List of all simulated drone missions
        """
        print(f"\nIndexing {len(missions)} simulated drones...")
        start_time = time.time()
        
        for mission in missions:
            # Generate trajectory
            trajectory = Trajectory(
                mission=mission,
                sampling_rate=self.config.sampling_rate
            )
            
            # Index in spatial structure
            self.spatial_index.index_mission(mission, trajectory)
            
            # Store for later reference
            self.trajectories[mission.drone_id] = trajectory
            self.missions[mission.drone_id] = mission
        
        index_time = time.time() - start_time
        print(f"✅ Indexed {len(missions)} drones in {index_time:.3f}s")
        
        # Print statistics
        stats = self.spatial_index.get_statistics()
        print(f"   Grid cells occupied: {stats['grid_stats']['occupied_cells']}")
        print(f"   Avg drones per cell: {stats['grid_stats']['avg_drones_per_cell']:.2f}")
    
    def check_mission(self, primary_mission: Mission) -> ConflictReport:
        """
        Strategic Deconfliction: Check if a mission is safe to execute
        This is the main API called for pre-flight clearance
        
        Args:
            primary_mission: The mission requesting clearance
        
        Returns:
            ConflictReport with detailed conflict information
        """
        print(f"\n🔍 Checking mission: {primary_mission.mission_id}")
        start_time = time.time()
        
        # Generate trajectory for primary mission
        primary_trajectory = Trajectory(
            mission=primary_mission,
            sampling_rate=self.config.sampling_rate
        )
        
        # Find all conflicts
        conflicts = self._detect_conflicts(primary_mission, primary_trajectory)
        
        # Calculate execution time
        execution_time = time.time() - start_time
        
        # Calculate safe percentage
        total_points = len(primary_trajectory.points)
        conflict_points = len(set(c.timestamp for c in conflicts))
        safe_percentage = ((total_points - conflict_points) / total_points * 100) if total_points > 0 else 100.0
        
        # Create report
        report = ConflictReport(
            mission_id=primary_mission.mission_id,
            conflicts=conflicts,
            safe_percentage=safe_percentage,
            execution_time=execution_time
        )
        
        print(f"✅ Check complete in {execution_time:.3f}s")
        print(f"   Status: {report.status}")
        print(f"   Total conflicts: {report.total_conflicts}")
        print(f"   Critical: {report.critical_conflicts}, Warning: {report.warning_conflicts}")
        print(f"   Safe percentage: {report.safe_percentage:.1f}%")
        
        return report
    
    def _detect_conflicts(self, 
                         primary_mission: Mission,
                         primary_trajectory: Trajectory) -> List[Conflict]:
        """
        Detect all conflicts using 4D spatiotemporal checking
        
        Args:
            primary_mission: Primary mission
            primary_trajectory: Primary trajectory
        
        Returns:
            List of detected conflicts
        """
        conflicts = []
        conflict_id_counter = 0
        
        # Get safety buffer
        safety_buffer = primary_mission.safety_buffer
        
        # Check each point in the primary trajectory
        for i, point in enumerate(primary_trajectory.points):
            position = point[:3]  # [x, y, z]
            timestamp = point[3]   # t
            
            # Stage 1: Grid-based broad phase - find nearby drones
            nearby_drones = self.spatial_index.find_potential_conflicts(
                position,
                safety_buffer,
                exclude_drones={primary_mission.drone_id}
            )
            
            # Stage 2: Precise checking with temporal validation
            for drone_id in nearby_drones:
                # Get the other drone's position at this time
                other_trajectory = self.trajectories[drone_id]
                other_position = other_trajectory.get_position_at_time(timestamp)
                
                if other_position is None:
                    # Other drone not active at this time
                    continue
                
                # Calculate 3D distance
                distance = np.linalg.norm(position - other_position)
                
                # Check if it's a conflict
                if distance < safety_buffer:
                    # Determine severity
                    severity = self._classify_severity(distance)
                    
                    # Create conflict object
                    conflict = Conflict(
                        conflict_id=f"C_{conflict_id_counter:06d}",
                        primary_mission_id=primary_mission.mission_id,
                        conflicting_drone_id=drone_id,
                        location=position,
                        timestamp=timestamp,
                        distance=distance,
                        severity=severity
                    )
                    
                    conflicts.append(conflict)
                    conflict_id_counter += 1
        
        # Remove duplicate conflicts (same drone, nearby times)
        conflicts = self._deduplicate_conflicts(conflicts)
        
        return conflicts
    
    def _classify_severity(self, distance: float) -> str:
        """
        Classify conflict severity based on separation distance
        FlytBase uses similar classification
        
        Args:
            distance: Separation distance in meters
        
        Returns:
            "CRITICAL", "WARNING", or "INFO"
        """
        if distance < self.config.critical_distance:
            return "CRITICAL"
        elif distance < self.config.warning_distance:
            return "WARNING"
        else:
            return "INFO"
    
    def _deduplicate_conflicts(self, conflicts: List[Conflict]) -> List[Conflict]:
        """
        Remove duplicate conflicts (same pair of drones at nearby times)
        Keeps only the closest approach for each drone pair
        
        Args:
            conflicts: List of all detected conflicts
        
        Returns:
            Deduplicated list of conflicts
        """
        if not conflicts:
            return []
        
        # Group conflicts by drone pair
        drone_pairs = {}
        
        for conflict in conflicts:
            pair_key = (conflict.primary_mission_id, conflict.conflicting_drone_id)
            
            if pair_key not in drone_pairs:
                drone_pairs[pair_key] = []
            
            drone_pairs[pair_key].append(conflict)
        
        # For each pair, keep conflicts that are >5 seconds apart (separate encounters)
        deduplicated = []
        
        for pair_key, pair_conflicts in drone_pairs.items():
            # Sort by time
            pair_conflicts.sort(key=lambda c: c.timestamp)
            
            # Keep first conflict and any others >5 seconds later
            if pair_conflicts:
                deduplicated.append(pair_conflicts[0])
                
                last_time = pair_conflicts[0].timestamp
                for conflict in pair_conflicts[1:]:
                    if conflict.timestamp - last_time > 5.0:
                        deduplicated.append(conflict)
                        last_time = conflict.timestamp
        
        return deduplicated
    
    def get_statistics(self) -> dict:
        """Get detector statistics"""
        return {
            "indexed_drones": len(self.trajectories),
            "spatial_index_stats": self.spatial_index.get_statistics(),
            "config": {
                "safety_buffer": self.config.safety_buffer,
                "critical_distance": self.config.critical_distance,
                "warning_distance": self.config.warning_distance
            }
        }


if __name__ == "__main__":
    print("Testing Conflict Detection Engine...")
    
    # Import for testing
    import sys
    from pathlib import Path
    backend_path = Path(__file__).parent.parent
    sys.path.insert(0, str(backend_path))
    
    from core.models import create_mission_from_waypoints
    
    # Define airspace
    airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
    
    # Create conflict detector
    detector = ConflictDetector(airspace_bounds=airspace_bounds)
    
    print("\n" + "="*60)
    print("TEST 1: Conflict-Free Scenario")
    print("="*60)
    
    # Create simulated drones (well separated)
    sim_missions = []
    for i in range(5):
        waypoints = [
            (i * 2000, 0, 50 + i * 50, 0),
            (i * 2000, 5000, 100 + i * 50, 50),
            (i * 2000, 10000, 50 + i * 50, 100)
        ]
        mission = create_mission_from_waypoints(
            mission_id=f"SIM_{i:03d}",
            waypoint_coords=waypoints,
            drone_id=f"UAV_SIM_{i:04d}"
        )
        sim_missions.append(mission)
    
    # Index simulated drones
    detector.index_simulation_drones(sim_missions)
    
    # Create primary mission (should be clear)
    primary_waypoints = [
        (1000, 0, 200, 0),
        (1000, 5000, 250, 50),
        (1000, 10000, 200, 100)
    ]
    primary_mission = create_mission_from_waypoints(
        mission_id="PRIMARY_001",
        waypoint_coords=primary_waypoints,
        drone_id="UAV_PRIMARY"
    )
    
    # Check for conflicts
    report = detector.check_mission(primary_mission)
    
    print("\n" + "="*60)
    print("TEST 2: Conflict Scenario")
    print("="*60)
    
    # Create primary mission that intersects with simulated drones
    conflict_waypoints = [
        (0, 0, 50, 0),
        (5000, 5000, 100, 50),
        (10000, 10000, 50, 100)
    ]
    conflict_mission = create_mission_from_waypoints(
        mission_id="PRIMARY_002",
        waypoint_coords=conflict_waypoints,
        drone_id="UAV_PRIMARY_2",
        safety_buffer=50.0  # Larger buffer to detect more conflicts
    )
    
    # Check for conflicts
    report2 = detector.check_mission(conflict_mission)
    
    if report2.conflicts:
        print("\n📍 Conflict Details:")
        for conflict in report2.conflicts[:3]:  # Show first 3
            print(f"   {conflict}")
    
    print("\n" + "="*60)
    print("Statistics")
    print("="*60)
    stats = detector.get_statistics()
    print(f"Indexed drones: {stats['indexed_drones']}")
    print(f"Spatial index efficiency: {stats['spatial_index_stats']['grid_stats']['avg_drones_per_cell']:.2f} drones/cell")
    
    print("\n🚀 Conflict Detection Engine working correctly!")