# tests/profile_system.py
"""
Profiling script for performance analysis
"""

import cProfile
import pstats
from pathlib import Path
import sys

backend_path = Path(__file__).parent.parent / "backend"
sys.path.insert(0, str(backend_path))

from core.conflict_detector import ConflictDetector
from core.models import create_mission_from_waypoints
from simulation.drone_generator import DroneTrajectoryGenerator, DroneGeneratorConfig


def profile_full_workflow():
    """Profile the complete workflow"""
    airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
    
    # Generate drones
    gen_config = DroneGeneratorConfig(num_drones=5000, airspace_bounds=airspace_bounds)
    generator = DroneTrajectoryGenerator(gen_config)
    missions = generator.generate_missions()
    
    # Initialize detector
    detector = ConflictDetector(airspace_bounds)
    detector.index_simulation_drones(missions)
    
    # Check mission
    primary = create_mission_from_waypoints(
        "PROFILE_001",
        [(5000, 5000, 200, 0), (6000, 6000, 250, 300)],
        "UAV_PRIMARY",
        safety_buffer=50.0
    )
    
    report = detector.check_mission(primary)
    print(f"Conflicts found: {report.total_conflicts}")


if __name__ == "__main__":
    print("Profiling UAV Deconfliction System...\n")
    
    profiler = cProfile.Profile()
    profiler.enable()
    
    profile_full_workflow()
    
    profiler.disable()
    
    # Save stats
    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    
    print("\n" + "="*60)
    print("TOP 20 FUNCTIONS BY CUMULATIVE TIME")
    print("="*60)
    stats.print_stats(20)
    
    # Save to file
    stats.dump_stats('tests/benchmarks/profile_stats.prof')
    print("\n📊 Profile saved to: tests/benchmarks/profile_stats.prof")