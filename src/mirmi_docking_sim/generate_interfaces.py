import os
import re
from pathlib import Path

# Konfiguration
OUTPUT_FILE = "INTERFACE_DOCUMENTATION_SMART.md"
ROOT_DIR = os.getcwd()

# --- WISSENSDATENBANK FÜR EXTERNE NODES ---
# Hier definieren wir Inputs/Outputs für Nodes, deren Code wir nicht haben.
# Format: 'executable_name': {'subs': [...], 'pubs': [...]}
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
        'pubs': ['/tf', '/tf_static']
    },
    # Hier können weitere Nodes ergänzt werden
}

# Regex Patterns
PATTERNS = {
    'node_class': re.compile(r'class\s+(\w+)\(Node\):'),
    'publisher': re.compile(r'create_publisher\s*\(\s*(\w+),\s*[\'"]([^\'"]+)[\'"]'),
    'subscription': re.compile(r'create_subscription\s*\(\s*(\w+),\s*[\'"]([^\'"]+)[\'"]'),
    'parameter': re.compile(r'declare_parameter\s*\(\s*[\'"]([^\'"]+)[\'"]\s*,\s*(.+)'),
    'launch_arg': re.compile(r'DeclareLaunchArgument\s*\(\s*[\'"]([^\'"]+)[\'"]'),
    'sdf_sensor': re.compile(r'<sensor\s+name=[\'"]([^\'"]+)[\'"]\s+type=[\'"]([^\'"]+)[\'"]>'),
    'sdf_topic': re.compile(r'<topic>(.*?)</topic>'),
}

def scan_files(root_path):
    data = {
        'py_nodes': [],
        'launch_nodes': [],
        'launch_args': [],
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

    node_info = {'file': path.name, 'class': class_match.group(1), 'pubs': [], 'subs': [], 'params': []}

    for match in PATTERNS['publisher'].findall(content):
        node_info['pubs'].append({'topic': match[1], 'type': match[0]})
        data['all_topics'].add(match[1])

    for match in PATTERNS['subscription'].findall(content):
        node_info['subs'].append({'topic': match[1], 'type': match[0]})
        data['all_topics'].add(match[1])

    for match in PATTERNS['parameter'].findall(content):
        val = match[1].split(')')[0].strip()
        node_info['params'].append({'name': match[0], 'default': val})

    data['py_nodes'].append(node_info)

def analyze_launch_file(path, content, data):
    args = PATTERNS['launch_arg'].findall(content)
    if args: data['launch_args'].append({'file': path.name, 'args': args})

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
            
            # Remappings: Wir speichern sie jetzt als Dictionary für leichteren Lookup
            # Format: internal -> external
            remaps = re.findall(r"\(\s*['\"]([^'\"]+)['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)", stripped)
            for r in remaps:
                current_node['remappings'][r[0]] = r[1]
                data['all_topics'].add(r[1])

            if brace_count <= 0:
                if current_node['package'] != '?': data['launch_nodes'].append(current_node)
                current_node = None

def analyze_sdf_file(path, content, data):
    sensors = []
    for match in PATTERNS['sdf_sensor'].finditer(content):
        name, type_ = match.group(1), match.group(2)
        start_pos = match.end()
        end_pos = content.find('</sensor>', start_pos)
        block = content[start_pos:end_pos]
        t_match = PATTERNS['sdf_topic'].search(block)
        topic = t_match.group(1) if t_match else "default"
        sensors.append({'name': name, 'type': type_, 'topic': topic})
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

def generate_mermaid(data):
    lines = ["```mermaid", "graph LR"]
    
    # Styles
    lines.append("    classDef rosNode fill:#d4edda,stroke:#28a745,stroke-width:2px;")
    lines.append("    classDef gzNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;")
    lines.append("    classDef topic fill:#e2e3e5,stroke:#6c757d,stroke-width:1px,rx:5,ry:5;")
    lines.append("    classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:2px,stroke-dasharray: 5 5;")

    # 1. ROS Python Nodes (wie gehabt)
    lines.append("\n    %% ROS Python Nodes")
    for node in data['py_nodes']:
        nid = clean_id(node['class'])
        lines.append(f"    {nid}({node['class']}):::rosNode")
        for pub in node['pubs']:
            lines.append(f"    {nid} --> {clean_id(pub['topic'])}([{pub['topic']}]):::topic")
        for sub in node['subs']:
            lines.append(f"    {clean_id(sub['topic'])} --> {nid}")

    # 2. Launch Nodes (SMART LOGIC)
    lines.append("\n    %% Launch Nodes (Smart)")
    for node in data['launch_nodes']:
        display_name = node['name'] if node['name'] != '?' else node['executable']
        nid = clean_id(display_name)
        lines.append(f"    {nid}({display_name}):::rosNode")
        
        exec_name = node['executable']
        
        # Fall A: Wir kennen den Node (Standard Interface vorhanden)
        if exec_name in STANDARD_INTERFACES:
            std = STANDARD_INTERFACES[exec_name]
            
            # Subscriptions abhandeln
            for internal_sub in std['subs']:
                # Wurde remapped? Wenn ja, nimm neuen Namen. Wenn nein, nimm Standard.
                topic = node['remappings'].get(internal_sub, internal_sub)
                # Zeichne Topic -> Node (Subs haben Input-Pfeile)
                lines.append(f"    {clean_id(topic)}([{topic}]):::topic --> {nid}")
            
            # Publishers abhandeln
            for internal_pub in std['pubs']:
                topic = node['remappings'].get(internal_pub, internal_pub)
                # Zeichne Node -> Topic (Pubs haben Output-Pfeile)
                lines.append(f"    {nid} --> {clean_id(topic)}([{topic}]):::topic")

        # Fall B: Unbekannter Node -> Fallback auf alte Logik (nur Remappings anzeigen)
        else:
            for internal, external in node['remappings'].items():
                lines.append(f"    {nid} -.-> {clean_id(external)}([{external}]):::topic")

    # 3. Gazebo & Bridge (wie gehabt)
    lines.append("\n    %% Gazebo & Bridge")
    lines.append(f"    GazeboSim(Gazebo Simulation):::gzNode")
    for model in data['sdf_models']:
        for s in model['sensors']:
            if s['topic'] and s['topic'] != 'default':
                lines.append(f"    GazeboSim -- {s['name']} --> {clean_id(s['topic'])}[[{s['topic']}]]:::gzNode")

    for b in data['bridge_config']['topics']:
        rid = clean_id(b['ros'])
        gid = clean_id(b['gz'])
        bridge_node = f"Bridge_{rid}_{gid}"
        
        if "GZ_TO_ROS" in b['dir'] or "BIDIRECTIONAL" in b['dir']:
            lines.append(f"    {gid} ==> {bridge_node}{{Bridge}}:::bridge ==> {rid}")
        if "ROS_TO_GZ" in b['dir'] or "BIDIRECTIONAL" in b['dir']:
            lines.append(f"    {rid} ==> {bridge_node}{{Bridge}}:::bridge ==> {gid}")

    lines.append("```")
    return "\n".join(lines)

def generate_markdown(data):
    lines = []
    lines.append("# Smart Interface Dokumentation")
    lines.append("> Mit erweitertem Wissen über Standard-Nodes (AprilTag, etc.).\n")
    
    lines.append("## Erkanntes Verhalten")
    lines.append("Das Skript nutzt eine Wissensdatenbank für folgende Launch-Executables:")
    for k in STANDARD_INTERFACES.keys():
        lines.append(f"- `{k}`")
    
    lines.append("\n## System Architektur")
    lines.append(generate_mermaid(data))
    return "\n".join(lines)

def main():
    print(f"Scanne: {ROOT_DIR} ...")
    data = scan_files(ROOT_DIR)
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write(generate_markdown(data))
    print(f"Fertig: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
