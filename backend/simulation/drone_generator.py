# backend/simulation/drone_generator.py
"""
Drone Trajectory Generator
Generates realistic flight paths for 4K-5K simulated drones
"""

import numpy as np
from typing import List, Tuple, Optional
import time
from dataclasses import dataclass

# Try relative import first, fall back to absolute
try:
    from ..core.models import Mission, Waypoint, create_mission_from_waypoints
except ImportError:
    import sys
    from pathlib import Path
    backend_path = Path(__file__).parent.parent
    sys.path.insert(0, str(backend_path))
    from core.models import Mission, Waypoint, create_mission_from_waypoints


@dataclass
class DroneGeneratorConfig:
    """Configuration for drone trajectory generation"""
    num_drones: int = 5000
    airspace_bounds: Tuple[Tuple[float, float, float], Tuple[float, float, float]] = (
        (0, 0, 0), (10000, 10000, 500)
    )
    waypoints_per_mission: Tuple[int, int] = (3, 8)  # (min, max)
    mission_duration: Tuple[float, float] = (60, 600)  # seconds (1-10 min)
    cruise_altitude: Tuple[float, float] = (50, 400)  # meters
    max_speed: float = 15.0  # m/s (typical drone speed)
    safety_buffer: float = 10.0  # meters
    seed: Optional[int] = 42  # for reproducibility


class DroneTrajectoryGenerator:
    """
    Generates realistic drone trajectories for simulation
    
    FlytBase approach:
    - Diverse flight patterns (delivery, surveillance, patrol)
    - Realistic waypoint distributions
    - Varying altitudes and speeds
    - Time-staggered missions
    """
    
    def __init__(self, config: Optional[DroneGeneratorConfig] = None):
        """
        Initialize trajectory generator
        
        Args:
            config: Generation configuration
        """
        self.config = config or DroneGeneratorConfig()
        
        # Set random seed for reproducibility
        if self.config.seed is not None:
            np.random.seed(self.config.seed)
        
        # Extract bounds
        self.min_bounds = np.array(self.config.airspace_bounds[0])
        self.max_bounds = np.array(self.config.airspace_bounds[1])
        self.airspace_size = self.max_bounds - self.min_bounds
        
        print(f"✅ Drone Generator initialized")
        print(f"   Airspace: {self.airspace_size[0]}x{self.airspace_size[1]}x{self.airspace_size[2]}m")
        print(f"   Target drones: {self.config.num_drones}")
    
    def generate_missions(self, num_drones: Optional[int] = None) -> List[Mission]:
        """
        Generate all drone missions
        
        Args:
            num_drones: Number of drones to generate (uses config if None)
        
        Returns:
            List of Mission objects
        """
        num_drones = num_drones or self.config.num_drones
        
        print(f"\n🚁 Generating {num_drones} drone missions...")
        start_time = time.time()
        
        missions = []
        
        # Generate different flight patterns
        pattern_distribution = {
            'point_to_point': 0.4,    # Direct A to B (delivery)
            'patrol': 0.3,             # Rectangular patrol pattern
            'survey': 0.2,             # Grid survey pattern
            'random': 0.1              # Random waypoints
        }
        
        pattern_counts = {
            pattern: int(num_drones * ratio)
            for pattern, ratio in pattern_distribution.items()
        }
        
        # Adjust for rounding
        pattern_counts['point_to_point'] += num_drones - sum(pattern_counts.values())
        
        mission_id = 0
        
        # Generate each pattern type
        for pattern_type, count in pattern_counts.items():
            for i in range(count):
                if pattern_type == 'point_to_point':
                    mission = self._generate_point_to_point(mission_id)
                elif pattern_type == 'patrol':
                    mission = self._generate_patrol(mission_id)
                elif pattern_type == 'survey':
                    mission = self._generate_survey(mission_id)
                else:  # random
                    mission = self._generate_random(mission_id)
                
                missions.append(mission)
                mission_id += 1
        
        generation_time = time.time() - start_time
        
        print(f"✅ Generated {len(missions)} missions in {generation_time:.2f}s")
        print(f"   Point-to-point: {pattern_counts['point_to_point']}")
        print(f"   Patrol: {pattern_counts['patrol']}")
        print(f"   Survey: {pattern_counts['survey']}")
        print(f"   Random: {pattern_counts['random']}")
        
        return missions
    
    def _generate_point_to_point(self, mission_id: int) -> Mission:
        """
        Generate a point-to-point mission (delivery style)
        Start -> End with optional intermediate waypoint
        """
        # Random start and end points
        start_pos = self._random_position()
        end_pos = self._random_position()
        
        # Random altitude (cruise altitude range)
        altitude = np.random.uniform(*self.config.cruise_altitude)
        
        # Mission duration based on distance
        distance = np.linalg.norm(end_pos - start_pos)
        duration = distance / self.config.max_speed * 1.5  # Add 50% buffer
        
        # Random start time (stagger missions)
        start_time = np.random.uniform(0, 1800)  # Within first 30 minutes
        
        waypoints = [
            (start_pos[0], start_pos[1], altitude, start_time),
            (end_pos[0], end_pos[1], altitude, start_time + duration)
        ]
        
        # 30% chance of intermediate waypoint
        if np.random.random() < 0.3:
            mid_pos = (start_pos + end_pos) / 2 + np.random.normal(0, 500, 2)
            mid_pos = np.clip(mid_pos, self.min_bounds[:2], self.max_bounds[:2])
            mid_time = start_time + duration / 2
            waypoints.insert(1, (mid_pos[0], mid_pos[1], altitude, mid_time))
        
        return create_mission_from_waypoints(
            mission_id=f"DRONE_{mission_id:05d}",
            waypoint_coords=waypoints,
            drone_id=f"UAV_{mission_id:05d}",
            safety_buffer=self.config.safety_buffer
        )
    
    def _generate_patrol(self, mission_id: int) -> Mission:
        """
        Generate a patrol mission (rectangular pattern)
        """
        # Random center point
        center = self._random_position()
        
        # Patrol area size (500-2000m)
        patrol_size = np.random.uniform(500, 2000)
        
        # Create rectangular patrol pattern
        altitude = np.random.uniform(*self.config.cruise_altitude)
        start_time = np.random.uniform(0, 1800)
        
        # 4 corners of rectangle
        corners = [
            center + np.array([-patrol_size/2, -patrol_size/2]),
            center + np.array([patrol_size/2, -patrol_size/2]),
            center + np.array([patrol_size/2, patrol_size/2]),
            center + np.array([-patrol_size/2, patrol_size/2])
        ]
        
        # Clip to airspace
        corners = [np.clip(c, self.min_bounds[:2], self.max_bounds[:2]) for c in corners]
        
        # Calculate times (perimeter / speed)
        perimeter = patrol_size * 4
        total_duration = perimeter / self.config.max_speed
        segment_duration = total_duration / 4
        
        waypoints = []
        for i, corner in enumerate(corners):
            waypoints.append((
                corner[0], corner[1], altitude,
                start_time + i * segment_duration
            ))
        
        # Return to start
        waypoints.append((
            corners[0][0], corners[0][1], altitude,
            start_time + total_duration
        ))
        
        return create_mission_from_waypoints(
            mission_id=f"DRONE_{mission_id:05d}",
            waypoint_coords=waypoints,
            drone_id=f"UAV_{mission_id:05d}",
            safety_buffer=self.config.safety_buffer
        )
    
    def _generate_survey(self, mission_id: int) -> Mission:
        """
        Generate a survey mission (grid pattern)
        """
        # Random survey area
        center = self._random_position()
        survey_size = np.random.uniform(1000, 3000)
        
        # Grid parameters
        num_legs = np.random.randint(3, 6)
        altitude = np.random.uniform(*self.config.cruise_altitude)
        start_time = np.random.uniform(0, 1800)
        
        # Create grid pattern (lawnmower)
        waypoints = []
        
        start_y = center[1] - survey_size / 2
        leg_spacing = survey_size / num_legs
        
        for i in range(num_legs):
            y = start_y + i * leg_spacing
            
            if i % 2 == 0:
                # Left to right
                x_start = center[0] - survey_size / 2
                x_end = center[0] + survey_size / 2
            else:
                # Right to left
                x_start = center[0] + survey_size / 2
                x_end = center[0] - survey_size / 2
            
            # Clip to bounds
            x_start = np.clip(x_start, self.min_bounds[0], self.max_bounds[0])
            x_end = np.clip(x_end, self.min_bounds[0], self.max_bounds[0])
            y = np.clip(y, self.min_bounds[1], self.max_bounds[1])
            
            # Calculate time
            leg_distance = abs(x_end - x_start)
            leg_time = leg_distance / self.config.max_speed
            
            waypoints.append((x_start, y, altitude, start_time + i * leg_time))
            waypoints.append((x_end, y, altitude, start_time + (i + 1) * leg_time))
        
        return create_mission_from_waypoints(
            mission_id=f"DRONE_{mission_id:05d}",
            waypoint_coords=waypoints,
            drone_id=f"UAV_{mission_id:05d}",
            safety_buffer=self.config.safety_buffer
        )
    
    def _generate_random(self, mission_id: int) -> Mission:
        """
        Generate a random waypoint mission
        """
        num_waypoints = np.random.randint(*self.config.waypoints_per_mission)
        
        waypoints = []
        start_time = np.random.uniform(0, 1800)
        current_time = start_time
        
        for i in range(num_waypoints):
            pos = self._random_position()
            altitude = np.random.uniform(*self.config.cruise_altitude)
            
            waypoints.append((pos[0], pos[1], altitude, current_time))
            
            # Calculate time to next waypoint
            if i < num_waypoints - 1:
                next_pos = self._random_position()
                distance = np.linalg.norm(next_pos - pos)
                duration = distance / self.config.max_speed
                current_time += duration
        
        return create_mission_from_waypoints(
            mission_id=f"DRONE_{mission_id:05d}",
            waypoint_coords=waypoints,
            drone_id=f"UAV_{mission_id:05d}",
            safety_buffer=self.config.safety_buffer
        )
    
    def _random_position(self) -> np.ndarray:
        """Generate a random 2D position within airspace"""
        return np.random.uniform(
            self.min_bounds[:2],
            self.max_bounds[:2]
        )
    
    def get_statistics(self, missions: List[Mission]) -> dict:
        """Get statistics about generated missions"""
        if not missions:
            return {}
        
        durations = [m.duration for m in missions]
        waypoint_counts = [len(m.waypoints) for m in missions]
        
        # Calculate altitude distribution
        altitudes = []
        for mission in missions:
            for wp in mission.waypoints:
                altitudes.append(wp.z)
        
        return {
            "total_missions": len(missions),
            "duration": {
                "mean": np.mean(durations),
                "min": np.min(durations),
                "max": np.max(durations),
                "std": np.std(durations)
            },
            "waypoints_per_mission": {
                "mean": np.mean(waypoint_counts),
                "min": np.min(waypoint_counts),
                "max": np.max(waypoint_counts)
            },
            "altitude": {
                "mean": np.mean(altitudes),
                "min": np.min(altitudes),
                "max": np.max(altitudes),
                "std": np.std(altitudes)
            }
        }


if __name__ == "__main__":
    print("Testing Drone Trajectory Generator...")
    print("="*60)
    
    # Test with smaller number first
    config = DroneGeneratorConfig(
        num_drones=100,  # Test with 100 first
        airspace_bounds=((0, 0, 0), (10000, 10000, 500)),
        seed=42
    )
    
    generator = DroneTrajectoryGenerator(config)
    
    # Generate missions
    missions = generator.generate_missions()
    
    # Get statistics
    print("\n" + "="*60)
    print("Statistics")
    print("="*60)
    stats = generator.get_statistics(missions)
    
    print(f"Total missions: {stats['total_missions']}")
    print(f"\nMission duration:")
    print(f"  Mean: {stats['duration']['mean']:.1f}s")
    print(f"  Range: {stats['duration']['min']:.1f}s - {stats['duration']['max']:.1f}s")
    print(f"\nWaypoints per mission:")
    print(f"  Mean: {stats['waypoints_per_mission']['mean']:.1f}")
    print(f"  Range: {stats['waypoints_per_mission']['min']} - {stats['waypoints_per_mission']['max']}")
    print(f"\nAltitude distribution:")
    print(f"  Mean: {stats['altitude']['mean']:.1f}m")
    print(f"  Range: {stats['altitude']['min']:.1f}m - {stats['altitude']['max']:.1f}m")
    
    # Show sample missions
    print("\n" + "="*60)
    print("Sample Missions (first 3)")
    print("="*60)
    for mission in missions[:3]:
        print(f"\n{mission}")
        print(f"  Waypoints:")
        for wp in mission.waypoints:
            print(f"    {wp}")
    
    print("\n" + "="*60)
    print("Testing with 5000 drones...")
    print("="*60)
    
    config_5k = DroneGeneratorConfig(num_drones=5000, seed=42)
    generator_5k = DroneTrajectoryGenerator(config_5k)
    
    missions_5k = generator_5k.generate_missions()
    stats_5k = generator_5k.get_statistics(missions_5k)
    
    print(f"\n✅ Successfully generated {stats_5k['total_missions']} missions!")
    print(f"   Memory efficient: ~{len(missions_5k) * 500 / 1024 / 1024:.1f}MB")
    
    print("\n🚀 Drone Generator working correctly!")