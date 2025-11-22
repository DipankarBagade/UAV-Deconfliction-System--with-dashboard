# tests/test_performance.py
"""
Performance benchmarks and profiling for UAV Deconfliction System
Tests scalability from 100 to 10,000 drones
"""

import pytest
import time
import psutil
import numpy as np
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import json

backend_path = Path(__file__).parent.parent / "backend"
sys.path.insert(0, str(backend_path))

from core.conflict_detector import ConflictDetector, ConflictDetectorConfig
from core.models import create_mission_from_waypoints
from simulation.drone_generator import DroneTrajectoryGenerator, DroneGeneratorConfig


class TestPerformanceBenchmarks:
    """Performance and scalability tests"""
    
    def test_scalability_10k_drones(self):
        """Benchmark: System performance with 10,000 drones"""
        print("\n" + "="*60)
        print("SCALABILITY TEST: 10,000 Drones")
        print("="*60)
        
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        
        # Generate 10K drones
        print("\n1. Generating 10,000 drone trajectories...")
        start_gen = time.time()
        
        gen_config = DroneGeneratorConfig(num_drones=10000, airspace_bounds=airspace_bounds)
        generator = DroneTrajectoryGenerator(gen_config)
        missions = generator.generate_missions()
        
        gen_time = time.time() - start_gen
        print(f"   ✅ Generation time: {gen_time:.2f}s")
        
        # Index drones
        print("\n2. Indexing 10,000 drones in spatial index...")
        start_index = time.time()
        
        detector = ConflictDetector(airspace_bounds)
        detector.index_simulation_drones(missions)
        
        index_time = time.time() - start_index
        print(f"   ✅ Indexing time: {index_time:.2f}s")
        
        # Memory usage
        process = psutil.Process()
        memory_mb = process.memory_info().rss / 1024 / 1024
        print(f"   ✅ Memory usage: {memory_mb:.1f} MB")
        
        # Run conflict detection
        print("\n3. Running conflict detection...")
        primary = create_mission_from_waypoints(
            "BENCH_001",
            [(5000, 5000, 200, 0), (5500, 5500, 250, 300)],
            "UAV_PRIMARY",
            safety_buffer=50.0
        )
        
        start_check = time.time()
        report = detector.check_mission(primary)
        check_time = time.time() - start_check
        
        print(f"   ✅ Conflict check time: {check_time:.3f}s")
        print(f"   ✅ Conflicts found: {report.total_conflicts}")
        
        # Performance assertions
        assert gen_time < 5.0, f"Generation too slow: {gen_time:.2f}s"
        assert index_time < 120.0, f"Indexing too slow: {index_time:.2f}s"
        assert check_time < 2.0, f"Detection too slow: {check_time:.3f}s"
        assert memory_mb < 2000, f"Memory usage too high: {memory_mb:.1f}MB"
        
        print("\n" + "="*60)
        print("✅ SCALABILITY TEST PASSED")
        print("="*60)
        
        return {
            "num_drones": 10000,
            "generation_time": gen_time,
            "indexing_time": index_time,
            "detection_time": check_time,
            "memory_mb": memory_mb,
            "conflicts": report.total_conflicts
        }
    
    def test_scaling_performance(self):
        """Benchmark: Performance scaling from 100 to 10,000 drones"""
        print("\n" + "="*60)
        print("SCALING PERFORMANCE TEST")
        print("="*60)
        
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        drone_counts = [100, 500, 1000, 2500, 5000, 10000]
        
        results = {
            "drone_counts": [],
            "indexing_times": [],
            "detection_times": [],
            "memory_usage": []
        }
        
        for num_drones in drone_counts:
            print(f"\n📊 Testing with {num_drones} drones...")
            
            # Generate drones
            gen_config = DroneGeneratorConfig(num_drones=num_drones, airspace_bounds=airspace_bounds)
            generator = DroneTrajectoryGenerator(gen_config)
            missions = generator.generate_missions()
            
            # Index
            start_index = time.time()
            detector = ConflictDetector(airspace_bounds)
            detector.index_simulation_drones(missions)
            index_time = time.time() - start_index
            
            # Check mission
            primary = create_mission_from_waypoints(
                f"SCALE_{num_drones}",
                [(5000, 5000, 200, 0), (6000, 6000, 250, 300)],
                "UAV_PRIMARY"
            )
            
            start_check = time.time()
            report = detector.check_mission(primary)
            check_time = time.time() - start_check
            
            # Memory
            process = psutil.Process()
            memory_mb = process.memory_info().rss / 1024 / 1024
            
            results["drone_counts"].append(num_drones)
            results["indexing_times"].append(index_time)
            results["detection_times"].append(check_time)
            results["memory_usage"].append(memory_mb)
            
            print(f"   Index: {index_time:.2f}s | Detection: {check_time:.3f}s | Memory: {memory_mb:.1f}MB")
        
        # Save results
        self._save_benchmark_results(results)
        
        # Generate performance graphs
        self._generate_performance_graphs(results)
        
        print("\n✅ Scaling test complete - graphs saved to tests/benchmarks/")
        
        return results
    
    def test_concurrent_requests(self):
        """Load test: Multiple concurrent mission checks"""
        import concurrent.futures
        
        print("\n" + "="*60)
        print("LOAD TEST: Concurrent Requests")
        print("="*60)
        
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        
        # Setup
        gen_config = DroneGeneratorConfig(num_drones=5000, airspace_bounds=airspace_bounds)
        generator = DroneTrajectoryGenerator(gen_config)
        missions = generator.generate_missions()
        
        detector = ConflictDetector(airspace_bounds)
        detector.index_simulation_drones(missions)
        
        # Create test missions
        test_missions = []
        for i in range(100):
            x, y = np.random.randint(0, 10000, 2)
            mission = create_mission_from_waypoints(
                f"LOAD_{i}",
                [(x, y, 150, 0), (x+1000, y+1000, 200, 300)],
                f"UAV_LOAD_{i}"
            )
            test_missions.append(mission)
        
        # Run concurrent checks
        print(f"\n🔄 Running 100 concurrent mission checks...")
        start = time.time()
        
        with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
            futures = [executor.submit(detector.check_mission, m) for m in test_missions]
            results = [f.result() for f in concurrent.futures.as_completed(futures)]
        
        total_time = time.time() - start
        avg_time = total_time / len(test_missions)
        
        print(f"   ✅ Total time: {total_time:.2f}s")
        print(f"   ✅ Average per request: {avg_time:.3f}s")
        print(f"   ✅ Throughput: {len(test_missions)/total_time:.1f} requests/second")
        
        assert avg_time < 1.0, f"Average request time too high: {avg_time:.3f}s"
        
        return {
            "num_requests": len(test_missions),
            "total_time": total_time,
            "avg_time": avg_time,
            "throughput": len(test_missions)/total_time
        }
    
    def _save_benchmark_results(self, results):
        """Save benchmark results to JSON"""
        Path("tests/benchmarks").mkdir(parents=True, exist_ok=True)
        
        with open("tests/benchmarks/performance_results.json", "w") as f:
            json.dump(results, f, indent=2)
    
    def _generate_performance_graphs(self, results):
        """Generate performance visualization graphs"""
        Path("tests/benchmarks").mkdir(parents=True, exist_ok=True)
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('UAV Deconfliction System - Performance Benchmarks', fontsize=16)
        
        # 1. Indexing time vs drones
        axes[0, 0].plot(results["drone_counts"], results["indexing_times"], 'b-o', linewidth=2)
        axes[0, 0].set_xlabel('Number of Drones')
        axes[0, 0].set_ylabel('Indexing Time (seconds)')
        axes[0, 0].set_title('Spatial Indexing Performance')
        axes[0, 0].grid(True, alpha=0.3)
        
        # 2. Detection time vs drones
        axes[0, 1].plot(results["drone_counts"], results["detection_times"], 'r-o', linewidth=2)
        axes[0, 1].set_xlabel('Number of Drones')
        axes[0, 1].set_ylabel('Detection Time (seconds)')
        axes[0, 1].set_title('Conflict Detection Performance')
        axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Memory usage
        axes[1, 0].plot(results["drone_counts"], results["memory_usage"], 'g-o', linewidth=2)
        axes[1, 0].set_xlabel('Number of Drones')
        axes[1, 0].set_ylabel('Memory Usage (MB)')
        axes[1, 0].set_title('Memory Consumption')
        axes[1, 0].grid(True, alpha=0.3)
        
        # 4. Complexity analysis
        drone_counts = np.array(results["drone_counts"])
        detection_times = np.array(results["detection_times"])
        
        # O(n) reference
        linear = detection_times[0] * (drone_counts / drone_counts[0])
        # O(n log n) reference
        nlogn = detection_times[0] * (drone_counts / drone_counts[0]) * np.log2(drone_counts / drone_counts[0])
        
        axes[1, 1].plot(drone_counts, detection_times, 'r-o', label='Actual', linewidth=2)
        axes[1, 1].plot(drone_counts, linear, 'b--', label='O(n)', alpha=0.5)
        axes[1, 1].plot(drone_counts, nlogn, 'g--', label='O(n log n)', alpha=0.5)
        axes[1, 1].set_xlabel('Number of Drones')
        axes[1, 1].set_ylabel('Detection Time (seconds)')
        axes[1, 1].set_title('Algorithm Complexity')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('tests/benchmarks/performance_graphs.png', dpi=150, bbox_inches='tight')
        print("   📊 Graphs saved: tests/benchmarks/performance_graphs.png")


if __name__ == "__main__":
    # Run benchmarks
    test = TestPerformanceBenchmarks()
    
    print("Running comprehensive performance benchmarks...\n")
    
    # Test 1: 10K drones
    result_10k = test.test_scalability_10k_drones()
    
    # Test 2: Scaling
    scaling_results = test.test_scaling_performance()
    
    # Test 3: Load test
    load_results = test.test_concurrent_requests()
    
    print("\n" + "="*60)
    print("ALL BENCHMARKS COMPLETE")
    print("="*60)
    print("\n📊 Results saved to: tests/benchmarks/")
    print("   - performance_results.json")
    print("   - performance_graphs.png")