# backend/core/models.py
"""
Core data models for UAV Strategic Deconfliction System
FlytBase-style architecture
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np
from datetime import datetime


@dataclass
class Waypoint:
    """
    Represents a single waypoint in 3D space with time
    FlytBase uses similar structure for trajectory intent
    """
    x: float  # meters
    y: float  # meters
    z: float  # meters (altitude)
    timestamp: float  # seconds since mission start
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array [x, y, z, t]"""
        return np.array([self.x, self.y, self.z, self.timestamp])
    
    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate 3D Euclidean distance to another waypoint"""
        return np.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def __repr__(self) -> str:
        return f"Waypoint(x={self.x:.1f}, y={self.y:.1f}, z={self.z:.1f}, t={self.timestamp:.1f})"


@dataclass
class Mission:
    """
    Represents a drone mission with waypoints and time window
    This is the "Trajectory Intent" in FlytBase terminology
    """
    mission_id: str
    waypoints: List[Waypoint]
    time_window: Tuple[float, float]  # (start_time, end_time)
    drone_id: str = "PRIMARY"
    max_speed: float = 15.0  # m/s (typical drone speed)
    safety_buffer: float = 10.0  # meters (FlytBase standard separation)
    
    def __post_init__(self):
        """Validate mission parameters"""
        if len(self.waypoints) < 2:
            raise ValueError("Mission must have at least 2 waypoints")
        
        if self.time_window[0] >= self.time_window[1]:
            raise ValueError("Invalid time window: start must be before end")
        
        # Ensure waypoints are sorted by timestamp
        self.waypoints.sort(key=lambda w: w.timestamp)
    
    @property
    def start_time(self) -> float:
        return self.time_window[0]
    
    @property
    def end_time(self) -> float:
        return self.time_window[1]
    
    @property
    def duration(self) -> float:
        return self.end_time - self.start_time
    
    def get_bounding_box(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get 4D bounding box (min and max coordinates)
        Returns: (min_coords, max_coords) where each is [x, y, z, t]
        """
        points = np.array([w.to_array() for w in self.waypoints])
        return points.min(axis=0), points.max(axis=0)
    
    def __repr__(self) -> str:
        return (f"Mission(id={self.mission_id}, drone={self.drone_id}, "
                f"waypoints={len(self.waypoints)}, "
                f"time=[{self.start_time:.1f}, {self.end_time:.1f}])")


@dataclass
class Trajectory:
    """
    Interpolated trajectory from waypoints
    Represents the actual flight path with dense sampling
    """
    mission: Mission
    points: np.ndarray = field(default=None)  # Nx4 array: [x, y, z, t]
    sampling_rate: float = 1.0  # seconds between samples
    
    def __post_init__(self):
        """Generate interpolated trajectory if not provided"""
        if self.points is None:
            self.points = self._interpolate_trajectory()
    
    def _interpolate_trajectory(self) -> np.ndarray:
        """
        Interpolate smooth trajectory between waypoints
        FlytBase uses similar 4D trajectory sampling
        """
        if len(self.mission.waypoints) < 2:
            # Single waypoint - return as is
            return np.array([self.mission.waypoints[0].to_array()])
        
        # Collect all waypoint arrays
        waypoint_arrays = [w.to_array() for w in self.mission.waypoints]
        
        # Generate time samples
        start_time = self.mission.waypoints[0].timestamp
        end_time = self.mission.waypoints[-1].timestamp
        time_samples = np.arange(start_time, end_time + self.sampling_rate, self.sampling_rate)
        
        # Interpolate each dimension
        interpolated_points = []
        
        for t in time_samples:
            # Find the two waypoints that bracket this time
            point = self._interpolate_at_time(t, waypoint_arrays)
            interpolated_points.append(point)
        
        return np.array(interpolated_points)
    
    def _interpolate_at_time(self, t: float, waypoint_arrays: List[np.ndarray]) -> np.ndarray:
        """
        Linear interpolation at a specific time
        """
        # Find bracketing waypoints
        for i in range(len(waypoint_arrays) - 1):
            t0 = waypoint_arrays[i][3]
            t1 = waypoint_arrays[i + 1][3]
            
            if t0 <= t <= t1:
                # Linear interpolation
                alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0
                p0 = waypoint_arrays[i][:3]  # [x, y, z]
                p1 = waypoint_arrays[i + 1][:3]
                
                interpolated_pos = p0 + alpha * (p1 - p0)
                return np.array([interpolated_pos[0], interpolated_pos[1], 
                               interpolated_pos[2], t])
        
        # If t is beyond last waypoint, return last waypoint
        last = waypoint_arrays[-1]
        return np.array([last[0], last[1], last[2], t])
    
    def get_position_at_time(self, t: float) -> Optional[np.ndarray]:
        """
        Get interpolated position at a specific time
        Returns [x, y, z] or None if time is outside trajectory
        """
        if t < self.points[0, 3] or t > self.points[-1, 3]:
            return None
        
        # Find closest time index
        time_diffs = np.abs(self.points[:, 3] - t)
        closest_idx = np.argmin(time_diffs)
        
        # If very close, return that point
        if time_diffs[closest_idx] < self.sampling_rate / 2:
            return self.points[closest_idx, :3]
        
        # Otherwise interpolate between two closest points
        if closest_idx > 0 and self.points[closest_idx, 3] > t:
            idx0, idx1 = closest_idx - 1, closest_idx
        elif closest_idx < len(self.points) - 1:
            idx0, idx1 = closest_idx, closest_idx + 1
        else:
            return self.points[closest_idx, :3]
        
        t0, t1 = self.points[idx0, 3], self.points[idx1, 3]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0
        
        pos = self.points[idx0, :3] + alpha * (self.points[idx1, :3] - self.points[idx0, :3])
        return pos
    
    def __len__(self) -> int:
        return len(self.points)
    
    def __repr__(self) -> str:
        return f"Trajectory(mission={self.mission.mission_id}, points={len(self.points)})"


@dataclass
class Conflict:
    """
    Represents a detected conflict between two drones
    FlytBase uses similar structure for conflict alerts
    """
    conflict_id: str
    primary_mission_id: str
    conflicting_drone_id: str
    location: np.ndarray  # [x, y, z]
    timestamp: float  # time of conflict
    distance: float  # separation distance in meters
    severity: str  # "CRITICAL", "WARNING", "INFO"
    
    def __post_init__(self):
        """Validate conflict data"""
        valid_severities = ["CRITICAL", "WARNING", "INFO"]
        if self.severity not in valid_severities:
            raise ValueError(f"Severity must be one of {valid_severities}")
    
    def to_dict(self) -> dict:
        """Convert to dictionary for API response"""
        return {
            "conflict_id": self.conflict_id,
            "primary_mission_id": self.primary_mission_id,
            "conflicting_drone_id": self.conflicting_drone_id,
            "location": {
                "x": float(self.location[0]),
                "y": float(self.location[1]),
                "z": float(self.location[2])
            },
            "timestamp": self.timestamp,
            "distance": self.distance,
            "severity": self.severity
        }
    
    def __repr__(self) -> str:
        return (f"Conflict({self.severity}: {self.primary_mission_id} ↔ "
                f"{self.conflicting_drone_id} at t={self.timestamp:.1f}s, "
                f"dist={self.distance:.2f}m)")


@dataclass
class ConflictReport:
    """
    Comprehensive report of all conflicts for a mission
    This is what the Strategic Deconfliction Service returns
    """
    mission_id: str
    status: str = "CLEAR"  # ← Add default value here
    conflicts: List[Conflict] = field(default_factory=list)
    total_conflicts: int = 0
    critical_conflicts: int = 0
    warning_conflicts: int = 0
    safe_percentage: float = 100.0
    execution_time: float = 0.0  # seconds
    
    def __post_init__(self):
        """Calculate statistics"""
        self.total_conflicts = len(self.conflicts)
        self.critical_conflicts = sum(1 for c in self.conflicts if c.severity == "CRITICAL")
        self.warning_conflicts = sum(1 for c in self.conflicts if c.severity == "WARNING")
        
        # Status is CLEAR only if no conflicts
        if self.total_conflicts > 0:
            self.status = "CONFLICT"
        else:
            self.status = "CLEAR"
    
    def to_dict(self) -> dict:
        """Convert to dictionary for API response"""
        return {
            "mission_id": self.mission_id,
            "status": self.status,
            "conflicts": [c.to_dict() for c in self.conflicts],
            "summary": {
                "total_conflicts": self.total_conflicts,
                "critical_conflicts": self.critical_conflicts,
                "warning_conflicts": self.warning_conflicts,
                "safe_percentage": self.safe_percentage
            },
            "execution_time": self.execution_time
        }
    
    def __repr__(self) -> str:
        return (f"ConflictReport(mission={self.mission_id}, status={self.status}, "
                f"conflicts={self.total_conflicts})")


# Utility functions for model creation

def create_mission_from_waypoints(
    mission_id: str,
    waypoint_coords: List[Tuple[float, float, float, float]],
    drone_id: str = "PRIMARY",
    safety_buffer: float = 10.0
) -> Mission:
    """
    Convenience function to create a mission from coordinate tuples
    
    Args:
        mission_id: Unique mission identifier
        waypoint_coords: List of (x, y, z, t) tuples
        drone_id: Drone identifier
        safety_buffer: Safety separation distance
    
    Returns:
        Mission object
    """
    waypoints = [Waypoint(x, y, z, t) for x, y, z, t in waypoint_coords]
    
    # Calculate time window from waypoints
    times = [w.timestamp for w in waypoints]
    time_window = (min(times), max(times))
    
    return Mission(
        mission_id=mission_id,
        waypoints=waypoints,
        time_window=time_window,
        drone_id=drone_id,
        safety_buffer=safety_buffer
    )


if __name__ == "__main__":
    # Test the models
    print("Testing Core Data Models...")
    
    # Create a simple mission
    waypoints_data = [
        (0, 0, 50, 0),      # Start
        (100, 100, 75, 10),  # Waypoint 1
        (200, 50, 100, 20),  # Waypoint 2
        (300, 200, 50, 30)   # End
    ]
    
    mission = create_mission_from_waypoints(
        mission_id="TEST_001",
        waypoint_coords=waypoints_data,
        drone_id="UAV_PRIMARY"
    )
    
    print(f"✅ Created: {mission}")
    print(f"   Duration: {mission.duration}s")
    print(f"   Waypoints: {len(mission.waypoints)}")
    
    # Create trajectory
    trajectory = Trajectory(mission=mission, sampling_rate=1.0)
    print(f"✅ Created: {trajectory}")
    print(f"   Sampled points: {len(trajectory)}")
    
    # Test position interpolation
    pos = trajectory.get_position_at_time(15.0)
    print(f"✅ Position at t=15s: {pos}")
    
    # Create a conflict (for testing)
    conflict = Conflict(
        conflict_id="C_001",
        primary_mission_id="TEST_001",
        conflicting_drone_id="UAV_5234",
        location=np.array([150, 75, 87]),
        timestamp=15.0,
        distance=5.5,
        severity="CRITICAL"
    )
    print(f"✅ Created: {conflict}")
    
    # Create conflict report
    report = ConflictReport(
        mission_id="TEST_001",
        conflicts=[conflict],
        execution_time=0.05
    )
    print(f"✅ Created: {report}")
    
    print("\n🚀 All models working correctly!")