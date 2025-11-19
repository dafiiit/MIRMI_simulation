# Complete System Architecture (with Hidden Dependencies)
```mermaid
graph LR
    classDef rosNode fill:#d4edda,stroke:#28a745,stroke-width:2px;
    classDef gzNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;
    classDef topic fill:#e2e3e5,stroke:#6c757d,stroke-width:1px,rx:5,ry:5;
    classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:2px,stroke-dasharray: 5 5;
    classDef tf fill:#e1bee7,stroke:#8e24aa,stroke-width:1px,rx:5,ry:5;

    %% Unified ROS Nodes

    %% Gazebo & Bridge
    GazeboSim(Gazebo Simulation):::gzNode
```