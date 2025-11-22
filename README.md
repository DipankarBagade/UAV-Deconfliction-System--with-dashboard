# UAV Strategic Deconfliction System

A high-performance, FlytBase-style strategic deconfliction service for UAV (drone) operations. This system enables pre-flight clearance by detecting potential conflicts between planned missions and existing airspace traffic through 4D spatiotemporal analysis.

## 🚀 Features

- **Strategic Deconfliction**: Pre-flight clearance for drone missions against simulated airspace traffic
- **4D Conflict Detection**: Spatiotemporal analysis considering position, altitude, and time
- **High Performance**: Handles 5000+ simulated drones with sub-second conflict detection
- **REST API**: FastAPI-based service with automatic documentation
- **Interactive Dashboard**: Streamlit web interface for mission planning and 3D visualization
- **Spatial Indexing**: Hybrid grid + KD-tree indexing for efficient nearest-neighbor queries
- **Configurable Safety**: Adjustable safety buffers and conflict severity levels
- **Comprehensive Testing**: Extensive test scenarios covering edge cases and performance benchmarks

## 🏗️ Architecture

The system follows a modular architecture inspired by FlytBase's deconfliction approach:

```
backend/
├── api/           # FastAPI REST service
├── core/          # Core conflict detection engine
│   ├── models.py          # Data models (Mission, Conflict, etc.)
│   ├── conflict_detector.py # 4D conflict detection logic
│   └── spatial_index.py   # Hybrid spatial indexing
├── simulation/    # Drone trajectory generation
├── utils/         # Configuration and utilities
└── dashboard.py   # Streamlit visualization interface

tests/             # Comprehensive test suite
docs/              # Documentation
data/              # Sample data and scenarios
```

### Core Components

- **Conflict Detector**: Implements 4D spatiotemporal conflict detection using safety buffers
- **Spatial Index**: Hybrid grid-based broad phase + KD-tree precise checking
- **Trajectory Engine**: Interpolates smooth 4D trajectories from waypoints
- **Simulation Generator**: Creates realistic drone traffic patterns for testing

## 📋 Requirements

- Python 3.13+
- Dependencies listed in `backend/requirements.txt`

## 🚀 Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/uav-deconfliction-system.git
   cd uav-deconfliction-system
   ```

2. **Install dependencies:**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. **Configure the system:**
   Edit `backend/config.yaml` to adjust airspace dimensions, safety buffers, and other parameters.

## 🎯 Usage

### Starting the API Server

```bash
cd backend
python api/main.py
```

The API will be available at `http://localhost:8000` with documentation at `http://localhost:8000/docs`.

### Starting the Dashboard

```bash
cd backend
streamlit run dashboard.py
```

Access the interactive dashboard at `http://localhost:8501`.

### API Endpoints

#### Initialize System
```http
POST /api/v1/initialize
Content-Type: application/json

{
  "num_drones": 5000,
  "airspace_x": 10000,
  "airspace_y": 10000,
  "airspace_z": 500,
  "safety_buffer": 10.0
}
```

#### Check Mission for Conflicts
```http
POST /api/v1/check_mission
Content-Type: application/json

{
  "mission_id": "MISSION_001",
  "drone_id": "UAV_PRIMARY",
  "waypoints": [
    {"x": 1000, "y": 1000, "z": 100, "timestamp": 0},
    {"x": 5000, "y": 5000, "z": 200, "timestamp": 300},
    {"x": 9000, "y": 9000, "z": 100, "timestamp": 600}
  ],
  "safety_buffer": 50.0
}
```

Response:
```json
{
  "mission_id": "MISSION_001",
  "status": "CLEAR",
  "conflicts": [],
  "summary": {
    "total_conflicts": 0,
    "critical_conflicts": 0,
    "warning_conflicts": 0,
    "safe_percentage": 100.0
  },
  "safe_percentage": 100.0,
  "execution_time": 0.123,
  "message": "✅ Mission APPROVED for execution. No conflicts detected."
}
```

## ⚙️ Configuration

The system is configured via `backend/config.yaml`. Key settings include:

- **Airspace Bounds**: Define the 3D operational volume
- **Safety Parameters**: Critical/warning distances and buffers
- **Simulation Settings**: Number of drones, trajectory patterns
- **Performance Tuning**: Spatial index parameters, sampling rates

Environment variables can override YAML settings (prefix with `UAV_`).

## 🧪 Testing

Run the test suite:

```bash
cd backend
pytest tests/
```

Test scenarios include:
- Spatial conflicts (parallel, intersecting, head-on)
- Temporal separation
- Altitude-based separation
- Performance benchmarks (5000+ drone scale)
- Edge cases and boundary conditions

View test scenarios in `tests/TEST_SCENARIOS.md`.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make your changes and add tests
4. Run tests: `pytest tests/`
5. Commit your changes: `git commit -am 'Add new feature'`
6. Push to the branch: `git push origin feature/your-feature`
7. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- Inspired by FlytBase's strategic deconfliction architecture
- Built with FastAPI, NumPy, SciPy, and other open-source libraries
- Designed for BVLOS (Beyond Visual Line of Sight) drone operations

## 📞 Support

For questions or issues:
- Open an issue on GitHub
- Check the API documentation at `/docs`
- Review the dashboard for interactive examples

---

**Note**: This system is designed for research and development purposes. For production deployment, additional security measures and regulatory compliance checks should be implemented.
