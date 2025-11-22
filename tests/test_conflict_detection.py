# tests/test_conflict_detection.py
"""
Test suite for UAV Deconfliction System
Tests spatial and temporal conflict detection
"""

import pytest
import numpy as np
import sys
from pathlib import Path

current_dir = Path(__file__).parent
backend_dir = current_dir.parent / "backend"
sys.path.insert(0, str(backend_dir))

# Add backend to path
backend_path = Path(__file__).parent.parent / "backend"
sys.path.insert(0, str(backend_path))

from core.models import create_mission_from_waypoints, Trajectory
from core.conflict_detector import ConflictDetector, ConflictDetectorConfig


class TestConflictDetection:
    """Test suite for conflict detection functionality"""
    
    @pytest.fixture
    def detector(self):
        """Create a conflict detector for testing"""
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        config = ConflictDetectorConfig(
            safety_buffer=10.0,
            critical_distance=5.0,
            warning_distance=15.0
        )
        return ConflictDetector(airspace_bounds, config)
    
    @pytest.fixture
    def sample_missions(self):
        """Create sample missions for testing"""
        missions = []
        
        # Mission 1: Simple straight line
        missions.append(create_mission_from_waypoints(
            mission_id="TEST_001",
            waypoint_coords=[
                (0, 0, 100, 0),
                (1000, 1000, 100, 100)
            ],
            drone_id="DRONE_001"
        ))
        
        # Mission 2: Parallel to Mission 1 (no conflict)
        missions.append(create_mission_from_waypoints(
            mission_id="TEST_002",
            waypoint_coords=[
                (0, 500, 100, 0),
                (1000, 1500, 100, 100)
            ],
            drone_id="DRONE_002"
        ))
        
        # Mission 3: Intersects Mission 1 (conflict!)
        missions.append(create_mission_from_waypoints(
            mission_id="TEST_003",
            waypoint_coords=[
                (0, 1000, 100, 0),
                (1000, 0, 100, 100)
            ],
            drone_id="DRONE_003"
        ))
        
        return missions
    
    def test_detector_initialization(self, detector):
        """Test that detector initializes correctly"""
        assert detector is not None
        assert detector.config.safety_buffer == 10.0
        assert len(detector.trajectories) == 0
    
    def test_mission_indexing(self, detector, sample_missions):
        """Test that missions are indexed correctly"""
        detector.index_simulation_drones(sample_missions)
        
        assert len(detector.trajectories) == 3
        assert len(detector.missions) == 3
        assert "DRONE_001" in detector.trajectories
    
    def test_no_conflict_scenario(self, detector, sample_missions):
        """Test scenario with no conflicts (parallel paths)"""
        # Index only Mission 2 (parallel, no conflict)
        detector.index_simulation_drones([sample_missions[1]])
        
        # Check Mission 1
        report = detector.check_mission(sample_missions[0])
        
        assert report.status == "CLEAR"
        assert report.total_conflicts == 0
        assert report.safe_percentage == 100.0
    
    def test_conflict_scenario(self, detector, sample_missions):
        """Test scenario with conflicts (intersecting paths)"""
        # Index Mission 3 (intersects with Mission 1)
        detector.index_simulation_drones([sample_missions[2]])
        
        # Check Mission 1
        report = detector.check_mission(sample_missions[0])
        
        assert report.status == "CONFLICT"
        assert report.total_conflicts > 0
        assert len(report.conflicts) > 0
    
    def test_safety_buffer_sensitivity(self, detector, sample_missions):
        """Test that increasing safety buffer detects more conflicts"""
        detector.index_simulation_drones([sample_missions[2]])
        
        # Test with different safety buffers
        mission = sample_missions[0]
        
        # Small buffer
        mission.safety_buffer = 5.0
        report_small = detector.check_mission(mission)
        
        # Large buffer
        mission.safety_buffer = 50.0
        report_large = detector.check_mission(mission)
        
        # Larger buffer should detect same or more conflicts
        assert report_large.total_conflicts >= report_small.total_conflicts
    
    def test_temporal_separation(self, detector):
        """Test that temporally separated missions don't conflict"""
        # Mission 1: Time 0-100
        mission1 = create_mission_from_waypoints(
            "TEMP_001",
            [(500, 500, 100, 0), (600, 600, 100, 100)],
            "DRONE_T1"
        )
        
        # Mission 2: Same location, different time 200-300
        mission2 = create_mission_from_waypoints(
            "TEMP_002",
            [(500, 500, 100, 200), (600, 600, 100, 300)],
            "DRONE_T2"
        )
        
        detector.index_simulation_drones([mission2])
        report = detector.check_mission(mission1)
        
        # Should be clear - no temporal overlap
        assert report.status == "CLEAR"
    
    def test_head_on_collision(self, detector):
        """Test head-on collision detection"""
        # Drone 1: Goes from A to B
        mission1 = create_mission_from_waypoints(
            "HEAD_001",
            [(0, 0, 100, 0), (1000, 0, 100, 100)],
            "DRONE_H1"
        )
        
        # Drone 2: Goes from B to A (head-on!)
        mission2 = create_mission_from_waypoints(
            "HEAD_002",
            [(1000, 0, 100, 0), (0, 0, 100, 100)],
            "DRONE_H2"
        )
        
        detector.index_simulation_drones([mission2])
        report = detector.check_mission(mission1)
        
        # Should detect conflict
        assert report.status == "CONFLICT"
        assert report.critical_conflicts > 0
    
    def test_hovering_drone(self, detector):
        """Test conflict with stationary/hovering drone"""
        # Stationary drone
        hovering = create_mission_from_waypoints(
            "HOVER_001",
            [(500, 500, 100, 0), (500, 500, 100, 600)],
            "DRONE_HOVER",
            safety_buffer=50.0
        )
        
        # Moving drone that passes through
        moving = create_mission_from_waypoints(
            "MOVING_001",
            [(0, 500, 100, 200), (1000, 500, 100, 400)],
            "DRONE_MOVING"
        )
        
        detector.index_simulation_drones([hovering])
        report = detector.check_mission(moving)
        
        # Should detect conflict
        assert report.status == "CONFLICT"
    
    def test_altitude_separation(self, detector):
        """Test that altitude separation prevents conflicts"""
        # Low altitude
        mission_low = create_mission_from_waypoints(
            "ALT_LOW",
            [(500, 500, 50, 0), (600, 600, 50, 100)],
            "DRONE_LOW"
        )
        
        # High altitude (same horizontal path)
        mission_high = create_mission_from_waypoints(
            "ALT_HIGH",
            [(500, 500, 200, 0), (600, 600, 200, 100)],
            "DRONE_HIGH"
        )
        
        detector.index_simulation_drones([mission_high])
        report = detector.check_mission(mission_low)
        
        # Should be clear if altitude difference > safety buffer
        # (depends on safety buffer setting)
        altitude_diff = 150  # 200 - 50
        if altitude_diff > detector.config.safety_buffer:
            assert report.status == "CLEAR"


class TestTrajectoryInterpolation:
    """Test trajectory generation and interpolation"""
    
    def test_trajectory_generation(self):
        """Test that trajectories are generated correctly"""
        mission = create_mission_from_waypoints(
            "TRAJ_001",
            [(0, 0, 100, 0), (1000, 1000, 100, 100)],
            "DRONE_TRAJ"
        )
        
        trajectory = Trajectory(mission, sampling_rate=1.0)
        
        assert len(trajectory.points) > 0
        assert trajectory.points[0, 3] == 0  # First timestamp
        assert trajectory.points[-1, 3] == 100  # Last timestamp
    
    def test_position_interpolation(self):
        """Test position interpolation at specific times"""
        mission = create_mission_from_waypoints(
            "INTERP_001",
            [(0, 0, 100, 0), (1000, 1000, 100, 100)],
            "DRONE_INTERP"
        )
        
        trajectory = Trajectory(mission, sampling_rate=1.0)
        
        # At t=50 (midpoint), should be near (500, 500, 100)
        pos = trajectory.get_position_at_time(50.0)
        
        assert pos is not None
        assert 400 < pos[0] < 600  # X around 500
        assert 400 < pos[1] < 600  # Y around 500


class TestSpatialIndex:
    """Test spatial indexing performance"""
    
    def test_grid_cell_assignment(self):
        """Test that drones are assigned to correct grid cells"""
        from core.spatial_index import SpatialGrid
        
        airspace_bounds = (np.array([0, 0, 0]), np.array([10000, 10000, 500]))
        grid = SpatialGrid(airspace_bounds, cell_size=100.0)
        
        # Test position assignment
        point = np.array([550, 550, 100])
        cell_id = grid._get_cell_id(point)
        
        # Should be in cell (5, 5, 2)
        assert cell_id == (5, 5, 2)
    
    def test_nearby_drone_search(self, detector, sample_missions):
        """Test that spatial index finds nearby drones efficiently"""
        detector.index_simulation_drones(sample_missions)
        
        # Search near origin
        test_point = np.array([100, 100, 100])
        nearby = detector.spatial_index.find_potential_conflicts(
            test_point,
            search_radius=500.0
        )
        
        # Should find at least one drone
        assert len(nearby) > 0


# Test runner configuration
if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])