import os
import re
from pathlib import Path

# Konfiguration
OUTPUT_FILE = "INTERFACE_DOCUMENTATION_FINAL.md"
ROOT_DIR = os.getcwd()

# --- 1. STANDARD INTERFACES (Wissen über externe Launch-Nodes) ---
STANDARD_INTERFACES = {
    'apriltag_node': {
        'subs': ['image_rect', 'camera_info'],
        'pubs': ['detections', 'tag_detections_image', 'tf']
    },
    'depthimage_to_laserscan_node': {
        'subs': ['image', 'camera_info'],
        'pubs': ['scan']
    },
    'static_transform_publisher': {
        'subs': [],
        'pubs': ['/tf_static'] # TF Static ist der Standard-Output hier
    },
    'ros_gz_bridge': { # Parameter Bridge oft auch unter diesem Namen
        'subs': [], 'pubs': [] 
    }
}

# --- 2. NODE ALIASES (Duplikate verhindern) ---
NODE_ALIASES = {
    'CameraInfoSync': 'camera_info_sync_node',
    'DockingController': 'docking_controller',
    'AprilTagVisualizer': 'apriltag_visualizer',
    'OdomToTFPublisher': 'odom_to_tf_publisher',
    'DockingTestRunner': 'automated_test_runner',
    'AprilTagPosePublisher': 'apriltag_pose_publisher',
    'PCLPosePublisher': 'pcl_pose_publisher',
    'DepthCameraInfoSync': 'depth_camera_info_sync'
}

# --- 3. PATTERNS (Jetzt mit TF und Actions) ---
PATTERNS = {
    'node_class': re.compile(r'class\s+(\w+)\(Node\):'),
    'publisher': re.compile(r'create_publisher\s*\(\s*(\w+),\s*[\'"]([^\'"]+)[\'"]'),
    'subscription': re.compile(r'create_subscription\s*\(\s*(\w+),\s*[\'"]([^\'"]+)[\'"]'),
    'parameter': re.compile(r'declare_parameter\s*\(\s*[\'"]([^\'"]+)[\'"]\s*,\s*(.+)'),
    
    # VERSTECKTE ROS 2 MAGIE
    'tf_broadcaster': re.compile(r'TransformBroadcaster\s*\('),         # -> Pub: /tf
    'tf_static_broadcaster': re.compile(r'StaticTransformBroadcaster\s*\('), # -> Pub: /tf_static
    'tf_listener': re.compile(r'TransformListener\s*\('),               # -> Sub: /tf, /tf_static
    'action_client': re.compile(r'ActionClient\s*\(\s*self,\s*\w+,\s*[\'"]([^\'"]+)[\'"]'), # -> Action
    'action_server': re.compile(r'ActionServer\s*\(\s*self,\s*\w+,\s*[\'"]([^\'"]+)[\'"]'), # -> Action

    # LAUNCH & SDF
    'launch_arg': re.compile(r'DeclareLaunchArgument\s*\(\s*[\'"]([^\'"]+)[\'"]'),
    'sdf_sensor': re.compile(r'<sensor\s+name=[\'"]([^\'"]+)[\'"]\s+type=[\'"]([^\'"]+)[\'"]>'),
    'sdf_topic': re.compile(r'<topic>(.*?)</topic>'),
}

def scan_files(root_path):
    data = {
        'py_nodes': [],
        'launch_nodes': [],
        'sdf_models': [],
        'bridge_config': {'topics': []},
        'all_topics': set()
    }

    for path in Path(root_path).rglob('*'):
        if any(x in path.parts for x in ['build', 'install', '.git', '__pycache__', 'log']):
            continue
        if path.is_dir(): continue

        try:
            with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
        except: continue

        if path.suffix == '.py' and 'launch' not in path.parts:
            analyze_python_node(path, content, data)
        
        if path.suffix == '.py' and 'launch' in path.parts:
            analyze_launch_file(path, content, data)

        if path.suffix == '.sdf':
            analyze_sdf_file(path, content, data)

        if path.name == 'bridge.yaml':
            analyze_bridge_config(path, content, data)

    return data

def analyze_python_node(path, content, data):
    class_match = PATTERNS['node_class'].search(content)
    if not class_match: return
    
    node_info = {'file': path.name, 'class': class_match.group(1), 'pubs': [], 'subs': [], 'actions': [], 'params': []}
    
    # Standard Pubs/Subs
    for match in PATTERNS['publisher'].findall(content):
        node_info['pubs'].append(match[1])
    for match in PATTERNS['subscription'].findall(content):
        node_info['subs'].append(match[1])

    # --- HIDDEN DEPENDENCIES FINDEN ---
    if PATTERNS['tf_broadcaster'].search(content):
        node_info['pubs'].append('/tf')
    
    if PATTERNS['tf_static_broadcaster'].search(content):
        node_info['pubs'].append('/tf_static')

    if PATTERNS['tf_listener'].search(content):
        node_info['subs'].append('/tf')
        node_info['subs'].append('/tf_static')

    # Actions (vereinfacht als Topics dargestellt)
    for match in PATTERNS['action_client'].findall(content):
        action_name = match
        # Actions nutzen 5 Topics, wir zeigen nur den logischen Namen
        node_info['pubs'].append(f"{action_name}/goal") 
        node_info['subs'].append(f"{action_name}/feedback")

    data['py_nodes'].append(node_info)

def analyze_launch_file(path, content, data):
    lines = content.split('\n')
    current_node = None
    brace_count = 0
    for line in lines:
        stripped = line.strip()
        if 'Node(' in stripped and 'import' not in stripped:
            current_node = {'package': '?', 'executable': '?', 'name': '?', 'remappings': {}, 'params': [], 'file': path.name}
            brace_count = stripped.count('(') - stripped.count(')')
        elif current_node:
            brace_count += stripped.count('(') - stripped.count(')')
            if "package=" in stripped: current_node['package'] = re.search(r"package=['\"]([^'\"]+)['\"]", stripped).group(1)
            if "executable=" in stripped: current_node['executable'] = re.search(r"executable=['\"]([^'\"]+)['\"]", stripped).group(1)
            if "name=" in stripped: 
                m = re.search(r"name=['\"]([^'\"]+)['\"]", stripped)
                if m: current_node['name'] = m.group(1)
            remaps = re.findall(r"\(\s*['\"]([^'\"]+)['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)", stripped)
            for r in remaps: current_node['remappings'][r[0]] = r[1]
            if brace_count <= 0:
                if current_node['package'] != '?': data['launch_nodes'].append(current_node)
                current_node = None

def analyze_sdf_file(path, content, data):
    sensors = []
    for match in PATTERNS['sdf_sensor'].finditer(content):
        t_match = PATTERNS['sdf_topic'].search(content[match.end():content.find('</sensor>', match.end())])
        topic = t_match.group(1) if t_match else "default"
        sensors.append({'name': match.group(1), 'type': match.group(2), 'topic': topic})
    if sensors: data['sdf_models'].append({'file': path.name, 'sensors': sensors})

def analyze_bridge_config(path, content, data):
    entries = content.split('- ros_topic_name:')
    for entry in entries[1:]:
        topic = re.search(r'^\s*["\']?([^"\']+)["\']?', entry)
        gz = re.search(r'gz_topic_name:\s*["\']?([^"\']+)["\']?', entry)
        d = re.search(r'direction:\s*(\w+)', entry)
        if topic:
            data['bridge_config']['topics'].append({
                'ros': topic.group(1).strip(),
                'gz': gz.group(1).strip() if gz else "?",
                'dir': d.group(1).strip() if d else "BIDIRECTIONAL"
            })

def clean_id(name):
    return name.replace('/', '_').replace('.', '_').replace('-', '_').strip('_')

def merge_nodes(data):
    merged = {}
    # Python
    for py_node in data['py_nodes']:
        target_name = NODE_ALIASES.get(py_node['class'], py_node['class'])
        if target_name not in merged:
            merged[target_name] = {'id': target_name, 'pubs': set(), 'subs': set(), 'remappings': {}}
        merged[target_name]['pubs'].update(py_node['pubs'])
        merged[target_name]['subs'].update(py_node['subs'])
    
    # Launch
    for ln_node in data['launch_nodes']:
        node_name = ln_node['name'] if ln_node['name'] != '?' else ln_node['executable']
        if node_name not in merged:
            merged[node_name] = {'id': node_name, 'pubs': set(), 'subs': set(), 'remappings': {}}
        
        # Standard Interfaces anwenden
        exec_name = ln_node['executable']
        if exec_name in STANDARD_INTERFACES:
            std = STANDARD_INTERFACES[exec_name]
            for s in std['subs']:
                merged[node_name]['subs'].add(ln_node['remappings'].get(s, s))
            for p in std['pubs']:
                merged[node_name]['pubs'].add(ln_node['remappings'].get(p, p))
        
        merged[node_name]['remappings'] = ln_node['remappings']
    return merged

def generate_mermaid(data):
    lines = ["```mermaid", "graph LR"]
    lines.append("    classDef rosNode fill:#d4edda,stroke:#28a745,stroke-width:2px;")
    lines.append("    classDef gzNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;")
    lines.append("    classDef topic fill:#e2e3e5,stroke:#6c757d,stroke-width:1px,rx:5,ry:5;")
    lines.append("    classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:2px,stroke-dasharray: 5 5;")
    lines.append("    classDef tf fill:#e1bee7,stroke:#8e24aa,stroke-width:1px,rx:5,ry:5;") # TF bekommt lila Farbe

    merged_nodes = merge_nodes(data)

    lines.append("\n    %% Unified ROS Nodes")
    for name, node in merged_nodes.items():
        nid = clean_id(name)
        lines.append(f"    {nid}({name}):::rosNode")
        
        for p in node['pubs']:
            tid = clean_id(p)
            style = ":::tf" if "tf" in p else ":::topic"
            lines.append(f"    {nid} --> {tid}([{p}]){style}")
        
        for s in node['subs']:
            tid = clean_id(s)
            style = ":::tf" if "tf" in s else ":::topic"
            lines.append(f"    {tid} --> {nid}")

        # Remappings visuals (gestrichelt)
        for internal, external in node['remappings'].items():
            if external not in node['pubs'] and external not in node['subs']:
                 lines.append(f"    {nid} -.-> {clean_id(external)}([{external}]):::topic")

    lines.append("\n    %% Gazebo & Bridge")
    lines.append(f"    GazeboSim(Gazebo Simulation):::gzNode")
    for model in data['sdf_models']:
        for s in model['sensors']:
            if s['topic'] and s['topic'] != 'default':
                lines.append(f"    GazeboSim -- {s['name']} --> {clean_id(s['topic'])}[[{s['topic']}]]:::gzNode")

    for b in data['bridge_config']['topics']:
        rid = clean_id(b['ros'])
        gid = clean_id(b['gz'])
        bridge = f"Bridge_{rid}_{gid}"
        # Nur anzeigen wenn Topics nicht leer
        if rid and gid:
            lines.append(f"    {gid} ==> {bridge}{{Bridge}}:::bridge ==> {rid}")
            lines.append(f"    {rid} ==> {bridge}{{Bridge}}:::bridge ==> {gid}")

    lines.append("```")
    return "\n".join(lines)

def main():
    print(f"Scanne: {ROOT_DIR} ...")
    data = scan_files(ROOT_DIR)
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write("# Complete System Architecture (with Hidden Dependencies)\n" + generate_mermaid(data))
    print(f"Fertig: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
