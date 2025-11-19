import os
import re
from pathlib import Path

# Konfiguration
OUTPUT_FILE = "INTERFACE_DOCUMENTATION_VISUAL.md"
ROOT_DIR = os.getcwd()

# Regex Patterns (wie zuvor)
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
            current_node = {'package': '?', 'executable': '?', 'name': '?', 'remappings': [], 'params': [], 'file': path.name}
            brace_count = stripped.count('(') - stripped.count(')')
        
        elif current_node:
            brace_count += stripped.count('(') - stripped.count(')')
            if "package=" in stripped: current_node['package'] = re.search(r"package=['\"]([^'\"]+)['\"]", stripped).group(1)
            if "executable=" in stripped: current_node['executable'] = re.search(r"executable=['\"]([^'\"]+)['\"]", stripped).group(1)
            if "name=" in stripped: 
                m = re.search(r"name=['\"]([^'\"]+)['\"]", stripped)
                if m: current_node['name'] = m.group(1)
            
            # Remappings: ('old', 'new')
            remaps = re.findall(r"\(\s*['\"]([^'\"]+)['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)", stripped)
            for r in remaps:
                current_node['remappings'].append(r) # Store tuple (internal, global)
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
    """Erstellt valide Mermaid-IDs aus Strings (ersetzt / durch _)"""
    return name.replace('/', '_').replace('.', '_').replace('-', '_').strip('_')

def generate_mermaid(data):
    """Generiert den Mermaid Graph Code"""
    lines = ["```mermaid", "graph LR"]
    
    # Styles
    lines.append("    classDef rosNode fill:#d4edda,stroke:#28a745,stroke-width:2px;")
    lines.append("    classDef gzNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;")
    lines.append("    classDef topic fill:#e2e3e5,stroke:#6c757d,stroke-width:1px,rx:5,ry:5;")
    lines.append("    classDef bridge fill:#f8d7da,stroke:#dc3545,stroke-width:2px,stroke-dasharray: 5 5;")

    # 1. ROS Nodes (Python)
    lines.append("\n    %% ROS Python Nodes")
    for node in data['py_nodes']:
        nid = clean_id(node['class'])
        lines.append(f"    {nid}({node['class']}):::rosNode")
        
        # Pubs
        for pub in node['pubs']:
            tid = clean_id(pub['topic'])
            lines.append(f"    {nid} --> {tid}([{pub['topic']}]):::topic")
        
        # Subs
        for sub in node['subs']:
            tid = clean_id(sub['topic'])
            lines.append(f"    {tid} --> {nid}")

    # 2. Launch Nodes (External)
    lines.append("\n    %% Launch Nodes")
    for node in data['launch_nodes']:
        # Name fallback
        display_name = node['name'] if node['name'] != '?' else node['executable']
        nid = clean_id(display_name)
        lines.append(f"    {nid}({display_name}):::rosNode")
        
        # Remappings als Verbindungen
        for internal, external in node['remappings']:
            tid = clean_id(external)
            # Heuristik: "image" oder "scan" ist oft Input (Sub), "cmd" Output (Pub)
            # Da wir es nicht sicher wissen, nutzen wir eine neutrale Linie, 
            # oder wir raten für Visualisierung. Hier: Durchgezogene Linie.
            lines.append(f"    {nid} -.-> {tid}([{external}]):::topic")

    # 3. Gazebo Simulation
    lines.append("\n    %% Gazebo")
    lines.append(f"    GazeboSim(Gazebo Simulation):::gzNode")
    for model in data['sdf_models']:
        for s in model['sensors']:
            if s['topic'] and s['topic'] != 'default':
                gz_tid = clean_id(s['topic'])
                # Sensor publiziert in Gazebo
                lines.append(f"    GazeboSim -- {s['name']} --> {gz_tid}[[{s['topic']}]]:::gzNode")

    # 4. Bridge
    lines.append("\n    %% Bridge")
    for b in data['bridge_config']['topics']:
        rid = clean_id(b['ros'])
        gid = clean_id(b['gz'])
        bridge_node = f"Bridge_{rid}_{gid}"
        
        # Verbindung Visualisieren
        # Gazebo Topic <--> Bridge <--> ROS Topic
        
        if "GZ_TO_ROS" in b['dir'] or "BIDIRECTIONAL" in b['dir']:
            lines.append(f"    {gid} ==> {bridge_node}{{Bridge}}:::bridge ==> {rid}")
        if "ROS_TO_GZ" in b['dir'] or "BIDIRECTIONAL" in b['dir']:
            lines.append(f"    {rid} ==> {bridge_node}{{Bridge}}:::bridge ==> {gid}")

    lines.append("```")
    return "\n".join(lines)

def generate_markdown(data):
    # ... (Alter Text Generierung Code, verkürzt für Übersichtlichkeit) ...
    # Wir nutzen den gleichen Text-Generator wie vorher, hängen aber Mermaid an.
    
    lines = []
    lines.append("# Interface Dokumentation & Visualisierung")
    lines.append("> Automatisch generiert aus Source Code, Launchfiles und Configs.\n")
    
    # Kurzer Text-Teil (Zusammenfassung)
    lines.append("## Übersicht der Nodes")
    for n in data['py_nodes']:
        lines.append(f"- **{n['class']}** (Python): {len(n['subs'])} Subs, {len(n['pubs'])} Pubs")
    for n in data['launch_nodes']:
        name = n['name'] if n['name'] != '?' else n['executable']
        lines.append(f"- **{name}** (Launch): {len(n['remappings'])} Remappings")
        
    lines.append("\n## Gazebo Bridge Mapping")
    lines.append("| ROS Topic | Gazebo Topic |")
    lines.append("|---|---|")
    for b in data['bridge_config']['topics']:
        lines.append(f"| `{b['ros']}` | `{b['gz']}` |")

    lines.append("\n## System Architektur (Mermaid Visualisierung)")
    lines.append("Das folgende Diagramm zeigt den Datenfluss. \n- **Grün**: ROS Nodes\n- **Gelb**: Gazebo Simulation\n- **Rot**: Bridge\n- **Grau**: ROS Topics\n")
    
    lines.append(generate_mermaid(data))
    
    return "\n".join(lines)

def main():
    print(f"Scanne und Visualisiere: {ROOT_DIR} ...")
    data = scan_files(ROOT_DIR)
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write(generate_markdown(data))
    print(f"Fertig! Datei erstellt: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
