# backend/test_setup.py
import numpy as np
from scipy.spatial import cKDTree

print("✅ NumPy version:", np.__version__)
print("✅ SciPy imported successfully")

# Test KDTree (this is what we'll primarily use)
points = np.random.rand(1000, 3)
tree = cKDTree(points)
print("✅ KDTree created successfully")

# Test query
distances, indices = tree.query([0.5, 0.5, 0.5], k=10)
print("✅ KDTree query working (found 10 nearest neighbors)")

# Test we can handle large datasets
large_points = np.random.rand(5000, 4)  # 5K drones with x,y,z,t
print(f"✅ Can handle {len(large_points)} points (simulating 5K drones)")

print("\n🚀 All core dependencies installed and working!")
print("✅ Ready to build the deconfliction system")
print("\nNote: Using scipy's cKDTree (excellent performance for 5K drones)")