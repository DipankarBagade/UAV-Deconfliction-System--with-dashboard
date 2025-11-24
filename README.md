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
uav-deconfliction-system/
├── backend/
│   ├── core/                      # Core algorithms
│   │   ├── models.py              # Data models
│   │   ├── spatial_index.py      # Grid + KD-tree
│   │   └── conflict_detector.py  # 4D detection
│   ├── api/                       # REST API
│   │   └── main.py               # FastAPI endpoints
│   ├── simulation/                # Drone simulation
│   │   └── drone_generator.py    # Trajectory generation
│   ├── dashboard.py               # Streamlit UI
│   ├── requirements.txt           # Dependencies
│   └── config.yaml               # Configuration
├── tests/                         # Test suite
│   ├── test_conflict_detection.py
│   └── test_performance.py
├── docs/                          # Documentation
│   ├── DESIGN_NOTES.md           # Architecture details
│   ├── FAILURE_MODES.md          # Failure analysis
│   └── RUNBOOK.md                # Operations guide
├── README.md                      # This file
```

### Core Components

- **Conflict Detector**: Implements 4D spatiotemporal conflict detection using safety buffers
- **Spatial Index**: Hybrid grid-based broad phase + KD-tree precise checking
- **Trajectory Engine**: Interpolates smooth 4D trajectories from waypoints
- **Simulation Generator**: Creates realistic drone traffic patterns for testing

## 📋 Design Notes

### Architecture Changes

The system follows a modular, FlytBase-inspired architecture designed for scalability and maintainability:

- **Modular Design**: Core components (models, conflict detection, spatial indexing) are decoupled from API and simulation layers
- **Hybrid Spatial Indexing**: Two-stage approach (grid broad-phase + KD-tree narrow-phase) enables efficient conflict detection for 5000+ drones
- **4D Trajectory Handling**: Time-aware conflict detection considers both spatial separation and temporal overlap
- **Configurable Safety**: Adjustable safety buffers and severity levels support different operational requirements
- **RESTful API**: FastAPI-based service with automatic OpenAPI documentation and CORS support
- **Interactive Dashboard**: Streamlit-based visualization for mission planning and 3D trajectory display

**Key Architectural Decisions:**
- **Spatial Partitioning**: 100m grid cells for broad-phase filtering, reducing computational complexity from O(n²) to O(n)
- **Trajectory Sampling**: 1-second sampling rate balances accuracy with performance
- **Conflict Deduplication**: Removes redundant conflicts within 5-second windows
- **Memory Management**: NumPy arrays for efficient numerical computations

### Failure Modes

The system is designed to handle various failure scenarios gracefully:

- **System Not Initialized**: API returns 503 error if `/api/v1/initialize` hasn't been called
- **High Load Degradation**: Performance degrades linearly with drone count; monitor execution times
- **Spatial Index Failures**: Grid/KD-tree corruption could cause false negatives; restart recommended
- **Trajectory Interpolation Errors**: Invalid waypoints may cause NaN values; validate input data
- **Memory Exhaustion**: Large airspace or high drone density may exceed memory limits
- **Network Timeouts**: Long-running conflict checks (>30s) may timeout; increase client timeouts
- **Configuration Errors**: Invalid YAML config may prevent startup; validate on load

**Monitoring Points:**
- Conflict detection execution time per mission
- Spatial index statistics (occupied cells, average drones per cell)
- Memory usage during initialization and peak load
- API response times and error rates

### Operational Runbook

#### Startup Procedure

1. **Environment Setup**:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

2. **Configuration**:
   - Edit `backend/config.yaml` for airspace dimensions and safety parameters
   - Verify Python 3.13+ and required dependencies

3. **System Initialization**:
   ```bash
   # Start API server
   python api/main.py

   # In another terminal, initialize system
   curl -X POST "http://localhost:8000/api/v1/initialize" \
        -H "Content-Type: application/json" \
        -d '{"num_drones": 5000, "airspace_x": 10000, "airspace_y": 10000, "airspace_z": 500}'
   ```

4. **Verification**:
   - Check `/health` endpoint returns healthy status
   - Verify `/api/v1/status` shows initialized state
   - Test with sample mission via `/api/v1/check_mission`

#### Monitoring & Maintenance

- **Health Checks**: Monitor `/health` and `/api/v1/status` endpoints
- **Performance Metrics**: Track execution times in API logs
- **Log Analysis**: Check for error patterns in stdout/stderr
- **Resource Usage**: Monitor CPU/memory during peak loads

#### Troubleshooting

- **Slow Performance**: Reduce `num_drones` or increase `sampling_rate` in config
- **Memory Issues**: Decrease airspace size or optimize grid cell size
- **False Conflicts**: Adjust `safety_buffer` and `critical_distance` parameters
- **API Errors**: Check system initialization status and input validation

#### Scaling Guidelines

- **Horizontal Scaling**: Deploy multiple instances behind load balancer
- **Vertical Scaling**: Increase CPU cores for parallel conflict detection
- **Data Partitioning**: Split large airspaces into regions
- **Caching**: Cache spatial indexes for frequently used airspace configurations

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
