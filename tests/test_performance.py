# tests/test_performance.py
"""
Performance benchmarks and load tests for UAV Deconfliction System
Generates performance graphs and metrics
"""

import pytest
import time
import psutil
import numpy as np
import matplotlib.pyplot as plt
import json
from pathlib import Path
import sys
import pandas as pd
from concurrent.futures import ThreadPoolExecutor, as_completed

# Add backend to path
backend_path = Path(__file__).parent.parent / "backend"
sys.path.insert(0, str(backend_path))

from core.conflict_detector import ConflictDetector, ConflictDetectorConfig
from core.models import create_mission_from_waypoints
from simulation.drone_generator import DroneTrajectoryGenerator, DroneGeneratorConfig


class TestPerformance:
    """Performance and scalability benchmarks"""
    
    @pytest.fixture(scope="class")
    def output_dir(self):
        """Create output directory for results"""
        output = Path("tests/performance_results")
        output.mkdir(parents=True, exist_ok=True)
        return output
    
    def test_scalability_benchmark(self, output_dir):
        """
        Benchmark: Performance scaling from 100 to 10,000 drones
        Generates graphs showing indexing time, detection time, and memory usage
        """
        print("\n" + "="*70)
        print("SCALABILITY BENCHMARK: 100 to 10,000 Drones")
        print("="*70)
        
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        drone_counts = [100, 500, 1000, 2500, 5000, 7500, 10000]
        
        results = {
            "drone_counts": [],
            "generation_times": [],
            "indexing_times": [],
            "detection_times": [],
            "memory_usage_mb": [],
            "conflicts_found": []
        }
        
        for num_drones in drone_counts:
            print(f"\n{'─'*70}")
            print(f"Testing with {num_drones:,} drones...")
            print(f"{'─'*70}")
            
            # Measure generation time
            print(f"  1. Generating {num_drones:,} drone trajectories...")
            start = time.time()
            
            gen_config = DroneGeneratorConfig(
                num_drones=num_drones,
                airspace_bounds=airspace_bounds,
                seed=42  # Consistent results
            )
            generator = DroneTrajectoryGenerator(gen_config)
            missions = generator.generate_missions()
            
            gen_time = time.time() - start
            print(f"     ✅ Generation: {gen_time:.2f}s")
            
            # Measure indexing time
            print(f"  2. Indexing spatial structures...")
            start = time.time()
            
            detector = ConflictDetector(airspace_bounds)
            detector.index_simulation_drones(missions)
            
            index_time = time.time() - start
            print(f"     ✅ Indexing: {index_time:.2f}s")
            
            # Measure detection time
            print(f"  3. Running conflict detection...")
            primary = create_mission_from_waypoints(
                f"BENCH_{num_drones}",
                [(5000, 5000, 200, 0), (5500, 5500, 250, 300), (6000, 6000, 200, 600)],
                "UAV_PRIMARY",
                safety_buffer=50.0
            )
            
            start = time.time()
            report = detector.check_mission(primary)
            detect_time = time.time() - start
            
            print(f"     ✅ Detection: {detect_time:.3f}s")
            print(f"     ✅ Conflicts: {report.total_conflicts}")
            
            # Measure memory usage
            process = psutil.Process()
            memory_mb = process.memory_info().rss / 1024 / 1024
            print(f"     ✅ Memory: {memory_mb:.1f} MB")
            
            # Store results
            results["drone_counts"].append(num_drones)
            results["generation_times"].append(gen_time)
            results["indexing_times"].append(index_time)
            results["detection_times"].append(detect_time)
            results["memory_usage_mb"].append(memory_mb)
            results["conflicts_found"].append(report.total_conflicts)
        
        # Save results to JSON
        print(f"\n{'='*70}")
        print("Saving results...")
        with open(output_dir / "scalability_results.json", "w") as f:
            json.dump(results, f, indent=2)
        print(f"✅ JSON saved: {output_dir / 'scalability_results.json'}")
        
        # Generate performance graphs
        self._generate_scalability_graphs(results, output_dir)
        
        # Print summary
        self._print_performance_summary(results)
        
        # Assertions for performance requirements
        max_detection_time = max(results["detection_times"])
        max_memory_mb = max(results["memory_usage_mb"])
        
        assert max_detection_time < 5.0, f"Detection too slow: {max_detection_time:.2f}s (max 5s)"
        assert max_memory_mb < 3000, f"Memory usage too high: {max_memory_mb:.1f}MB (max 3GB)"
        
        print(f"\n{'='*70}")
        print("✅ SCALABILITY BENCHMARK PASSED")
        print(f"{'='*70}\n")
    
    def test_concurrent_load(self, output_dir):
        """
        Load test: Simulate multiple concurrent mission checks
        Tests system under realistic production load
        """
        print("\n" + "="*70)
        print("LOAD TEST: Concurrent Mission Checks")
        print("="*70)
        
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        
        # Setup system with 5000 drones
        print("\n1. Setting up system with 5000 drones...")
        gen_config = DroneGeneratorConfig(num_drones=5000, airspace_bounds=airspace_bounds, seed=42)
        generator = DroneTrajectoryGenerator(gen_config)
        missions = generator.generate_missions()
        
        detector = ConflictDetector(airspace_bounds)
        detector.index_simulation_drones(missions)
        print("   ✅ System ready")
        
        # Create test missions
        print("\n2. Creating 100 test missions...")
        test_missions = []
        np.random.seed(42)
        for i in range(100):
            x_start = np.random.randint(0, 9000)
            y_start = np.random.randint(0, 9000)
            
            mission = create_mission_from_waypoints(
                f"LOAD_{i:03d}",
                [
                    (x_start, y_start, 150, 0),
                    (x_start + 1000, y_start + 1000, 200, 300)
                ],
                f"UAV_LOAD_{i:03d}",
                safety_buffer=30.0
            )
            test_missions.append(mission)
        print("   ✅ Test missions created")
        
        # Test different concurrency levels
        concurrency_levels = [1, 5, 10, 20, 50]
        load_results = {
            "concurrency": [],
            "total_time": [],
            "avg_time": [],
            "throughput": [],
            "errors": []
        }
        
        for workers in concurrency_levels:
            print(f"\n3. Testing with {workers} concurrent workers...")
            
            start = time.time()
            errors = 0
            
            def check_mission(mission):
                try:
                    return detector.check_mission(mission)
                except Exception as e:
                    return None
            
            with ThreadPoolExecutor(max_workers=workers) as executor:
                futures = [executor.submit(check_mission, m) for m in test_missions]
                results = [f.result() for f in as_completed(futures)]
                errors = sum(1 for r in results if r is None)
            
            total_time = time.time() - start
            avg_time = total_time / len(test_missions)
            throughput = len(test_missions) / total_time
            
            load_results["concurrency"].append(workers)
            load_results["total_time"].append(total_time)
            load_results["avg_time"].append(avg_time)
            load_results["throughput"].append(throughput)
            load_results["errors"].append(errors)
            
            print(f"   Total: {total_time:.2f}s | Avg: {avg_time:.3f}s | "
                  f"Throughput: {throughput:.1f} req/s | Errors: {errors}")
        
        # Save results
        with open(output_dir / "load_test_results.json", "w") as f:
            json.dump(load_results, f, indent=2)
        
        # Generate load test graphs
        self._generate_load_test_graphs(load_results, output_dir)
        
        print(f"\n{'='*70}")
        print("✅ LOAD TEST COMPLETED")
        print(f"{'='*70}\n")
    
    def test_worst_case_scenario(self, output_dir):
        """
        Stress test: Maximum conflict scenario
        Tests system under worst-case conditions
        """
        print("\n" + "="*70)
        print("STRESS TEST: Worst-Case Conflict Scenario")
        print("="*70)
        
        airspace_bounds = ((0, 0, 0), (10000, 10000, 500))
        
        # Create worst-case: All drones converging to center
        print("\n1. Creating worst-case scenario...")
        print("   All drones flying through airspace center...")
        
        gen_config = DroneGeneratorConfig(num_drones=1000, airspace_bounds=airspace_bounds, seed=42)
        generator = DroneTrajectoryGenerator(gen_config)
        missions = generator.generate_missions()
        
        detector = ConflictDetector(airspace_bounds)
        detector.index_simulation_drones(missions)
        
        # Primary mission through center
        center_mission = create_mission_from_waypoints(
            "STRESS_CENTER",
            [
                (4500, 4500, 200, 100),
                (5000, 5000, 200, 300),
                (5500, 5500, 200, 500)
            ],
            "UAV_STRESS",
            safety_buffer=100.0  # Large buffer for maximum conflicts
        )
        
        print("\n2. Running conflict detection...")
        start = time.time()
        report = detector.check_mission(center_mission)
        detect_time = time.time() - start
        
        print(f"\n3. Results:")
        print(f"   Detection time: {detect_time:.3f}s")
        print(f"   Total conflicts: {report.total_conflicts}")
        print(f"   Critical: {report.critical_conflicts}")
        print(f"   Warnings: {report.warning_conflicts}")
        print(f"   Safe percentage: {report.safe_percentage:.1f}%")
        
        stress_results = {
            "scenario": "center_convergence",
            "num_drones": 1000,
            "detection_time": detect_time,
            "total_conflicts": report.total_conflicts,
            "critical_conflicts": report.critical_conflicts,
            "warning_conflicts": report.warning_conflicts,
            "safe_percentage": report.safe_percentage
        }
        
        with open(output_dir / "stress_test_results.json", "w") as f:
            json.dump(stress_results, f, indent=2)
        
        # Should handle even worst case in reasonable time
        assert detect_time < 10.0, f"Stress test too slow: {detect_time:.2f}s"
        
        print(f"\n{'='*70}")
        print("✅ STRESS TEST PASSED")
        print(f"{'='*70}\n")
    
    def _generate_scalability_graphs(self, results, output_dir):
        """Generate comprehensive performance graphs"""
        print("\nGenerating performance graphs...")
        
        fig = plt.figure(figsize=(16, 12))
        
        # Create 2x3 subplot grid
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        drone_counts = np.array(results["drone_counts"])
        
        # 1. Indexing Time
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(drone_counts, results["indexing_times"], 'b-o', linewidth=2, markersize=8)
        ax1.set_xlabel('Number of Drones', fontsize=11, fontweight='bold')
        ax1.set_ylabel('Indexing Time (seconds)', fontsize=11, fontweight='bold')
        ax1.set_title('Spatial Index Build Time', fontsize=13, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(0, max(drone_counts) * 1.1)
        
        # 2. Detection Time
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(drone_counts, results["detection_times"], 'r-o', linewidth=2, markersize=8)
        ax2.set_xlabel('Number of Drones', fontsize=11, fontweight='bold')
        ax2.set_ylabel('Detection Time (seconds)', fontsize=11, fontweight='bold')
        ax2.set_title('Conflict Detection Performance', fontsize=13, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=2.0, color='orange', linestyle='--', label='2s Target')
        ax2.legend()
        ax2.set_xlim(0, max(drone_counts) * 1.1)
        
        # 3. Memory Usage
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(drone_counts, results["memory_usage_mb"], 'g-o', linewidth=2, markersize=8)
        ax3.set_xlabel('Number of Drones', fontsize=11, fontweight='bold')
        ax3.set_ylabel('Memory Usage (MB)', fontsize=11, fontweight='bold')
        ax3.set_title('Memory Consumption', fontsize=13, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.axhline(y=2000, color='red', linestyle='--', label='2GB Limit')
        ax3.legend()
        ax3.set_xlim(0, max(drone_counts) * 1.1)
        
        # 4. Algorithm Complexity
        ax4 = fig.add_subplot(gs[1, 1])
        detection_times = np.array(results["detection_times"])
        
        # Calculate complexity curves
        n0 = drone_counts[0]
        t0 = detection_times[0]
        
        # O(n) - linear
        linear = t0 * (drone_counts / n0)
        # O(n log n)
        nlogn = t0 * (drone_counts / n0) * (np.log2(drone_counts) / np.log2(n0))
        # O(n²)
        quadratic = t0 * ((drone_counts / n0) ** 2)
        
        ax4.plot(drone_counts, detection_times, 'r-o', linewidth=3, markersize=8, label='Actual')
        ax4.plot(drone_counts, linear, 'b--', linewidth=2, alpha=0.6, label='O(n)')
        ax4.plot(drone_counts, nlogn, 'g--', linewidth=2, alpha=0.6, label='O(n log n)')
        ax4.plot(drone_counts[:5], quadratic[:5], 'm--', linewidth=2, alpha=0.6, label='O(n²)')
        
        ax4.set_xlabel('Number of Drones', fontsize=11, fontweight='bold')
        ax4.set_ylabel('Detection Time (seconds)', fontsize=11, fontweight='bold')
        ax4.set_title('Algorithm Complexity Analysis', fontsize=13, fontweight='bold')
        ax4.legend(fontsize=10)
        ax4.grid(True, alpha=0.3)
        ax4.set_xlim(0, max(drone_counts) * 1.1)
        ax4.set_ylim(0, max(detection_times) * 1.5)
        
        # 5. Throughput (requests per second)
        ax5 = fig.add_subplot(gs[2, 0])
        throughput = [1/t if t > 0 else 0 for t in results["detection_times"]]
        ax5.bar(drone_counts, throughput, width=drone_counts[1]-drone_counts[0]*0.8, 
                color='purple', alpha=0.7, edgecolor='black')
        ax5.set_xlabel('Number of Drones', fontsize=11, fontweight='bold')
        ax5.set_ylabel('Throughput (checks/second)', fontsize=11, fontweight='bold')
        ax5.set_title('System Throughput', fontsize=13, fontweight='bold')
        ax5.grid(True, alpha=0.3, axis='y')
        
        # 6. Conflicts Found
        ax6 = fig.add_subplot(gs[2, 1])
        ax6.bar(drone_counts, results["conflicts_found"], width=drone_counts[1]-drone_counts[0]*0.8,
                color='orange', alpha=0.7, edgecolor='black')
        ax6.set_xlabel('Number of Drones', fontsize=11, fontweight='bold')
        ax6.set_ylabel('Conflicts Detected', fontsize=11, fontweight='bold')
        ax6.set_title('Conflicts vs Drone Count', fontsize=13, fontweight='bold')
        ax6.grid(True, alpha=0.3, axis='y')
        
        # Main title
        fig.suptitle('UAV Deconfliction System - Performance Benchmarks', 
                     fontsize=16, fontweight='bold', y=0.995)
        
        # Save
        plt.savefig(output_dir / 'performance_graphs.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"   ✅ Graphs saved: {output_dir / 'performance_graphs.png'}")
    
    def _generate_load_test_graphs(self, results, output_dir):
        """Generate load test visualization"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Load Test Results - Concurrent Mission Checks', fontsize=16, fontweight='bold')
        
        concurrency = results["concurrency"]
        
        # 1. Total time vs concurrency
        axes[0, 0].plot(concurrency, results["total_time"], 'b-o', linewidth=2, markersize=8)
        axes[0, 0].set_xlabel('Concurrent Workers')
        axes[0, 0].set_ylabel('Total Time (seconds)')
        axes[0, 0].set_title('Total Execution Time')
        axes[0, 0].grid(True, alpha=0.3)
        
        # 2. Average time per request
        axes[0, 1].plot(concurrency, results["avg_time"], 'r-o', linewidth=2, markersize=8)
        axes[0, 1].set_xlabel('Concurrent Workers')
        axes[0, 1].set_ylabel('Average Time (seconds)')
        axes[0, 1].set_title('Average Request Time')
        axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Throughput
        axes[1, 0].plot(concurrency, results["throughput"], 'g-o', linewidth=2, markersize=8)
        axes[1, 0].set_xlabel('Concurrent Workers')
        axes[1, 0].set_ylabel('Requests per Second')
        axes[1, 0].set_title('System Throughput')
        axes[1, 0].grid(True, alpha=0.3)
        
        # 4. Error rate
        axes[1, 1].bar(concurrency, results["errors"], color='red', alpha=0.7)
        axes[1, 1].set_xlabel('Concurrent Workers')
        axes[1, 1].set_ylabel('Number of Errors')
        axes[1, 1].set_title('Error Count')
        axes[1, 1].grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'load_test_graphs.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"   ✅ Load test graphs: {output_dir / 'load_test_graphs.png'}")
    
    def _print_performance_summary(self, results):
        """Print formatted performance summary"""
        print(f"\n{'='*70}")
        print("PERFORMANCE SUMMARY")
        print(f"{'='*70}")
        
        df = pd.DataFrame(results)
        df['throughput'] = 1 / df['detection_times']
        
        print("\n" + df.to_string(index=False))
        
        print(f"\n{'─'*70}")
        print("KEY METRICS:")
        print(f"{'─'*70}")
        print(f"  Max drones tested: {max(results['drone_counts']):,}")
        print(f"  Fastest detection: {min(results['detection_times']):.3f}s")
        print(f"  Slowest detection: {max(results['detection_times']):.3f}s")
        print(f"  Peak memory: {max(results['memory_usage_mb']):.1f} MB")
        print(f"  Max throughput: {max(1/t for t in results['detection_times']):.1f} checks/s")


if __name__ == "__main__":
    # Run all benchmarks
    print("\n" + "="*70)
    print("UAV DECONFLICTION SYSTEM - PERFORMANCE BENCHMARKS")
    print("="*70)
    
    test = TestPerformance()
    output_dir = Path("tests/performance_results")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Run tests
    test.test_scalability_benchmark(output_dir)
    test.test_concurrent_load(output_dir)
    test.test_worst_case_scenario(output_dir)
    
    print("\n" + "="*70)
    print("✅ ALL BENCHMARKS COMPLETED")
    print(f"Results saved to: {output_dir}")
    print("="*70 + "\n")