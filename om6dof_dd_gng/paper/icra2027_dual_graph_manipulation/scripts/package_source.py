#!/usr/bin/env python3
"""Package an editable manuscript without ROS build products or old papers."""
from pathlib import Path
import hashlib
import json
import zipfile

paper=Path(__file__).resolve().parents[1]
repo=paper.parents[2]
paths=[
 'experiments/cartesian_workspace/cpp/scan.cpp',
 'om6dof_dd_gng/include/om6dof_dd_gng/dynamic_density_gng.hpp',
 'om6dof_dd_gng/include/om6dof_dd_gng/semantic_target_selection.hpp',
 'om6dof_dd_gng/include/om6dof_dd_gng/semantic_cluster_propagation.hpp',
 'om6dof_dd_gng/include/om6dof_dd_gng/reachability_graph.hpp',
 'om6dof_dd_gng/src/topo_gng_node.cpp',
 'om6dof_dd_gng/src/reachability_graph_node.cpp',
 'om6dof_dd_gng/launch/ddgng_experiment_reachability.launch.py',
 'om6dof_dd_gng/config/topo_gng_v2.yaml',
 'om6dof_pick_and_place/om6dof_pick_and_place/graph_pick_node.py',
 'om6dof_pick_and_place/config/graph_pick.yaml',
]
manifest=[]
for rel in paths:
    f=repo/rel
    manifest.append({'repository_relative_path':rel,
                     'sha256':hashlib.sha256(f.read_bytes()).hexdigest()})
(paper/'data/manuscript_source_manifest.json').write_text(json.dumps({
    'scope':'Source inspected for this manuscript, not a record of live runtime state',
    'files':manifest},indent=2)+'\n')
root_files={'main.tex','main.pdf','main.bbl','references.bib','ieeeconf.cls','IEEEtran.bst',
            'build.sh','README.md','EVALUATION_PLAN.md','SUBMISSION_NOTES.md',
            'verification.json','evidence_audit.md','method_audit.md','literature_notes.md'}
folder_extensions={'.py','.pdf','.png','.json','.csv','.cls','.bst'}
files=[]
for p in paper.rglob('*'):
    if not p.is_file():continue
    rel=p.relative_to(paper)
    if len(rel.parts)==1 and p.name in root_files:
        files.append(p)
    elif rel.parts[0] in {'figures','data','scripts','template'} and p.suffix in folder_extensions:
        if '__pycache__' not in rel.parts: files.append(p)
with zipfile.ZipFile(paper/'paper_source.zip','w',compression=zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files): z.write(p,p.relative_to(paper))
with zipfile.ZipFile(paper/'paper_source.zip') as z:
    assert z.testzip() is None
    assert {'main.tex','main.pdf','ieeeconf.cls','IEEEtran.bst','references.bib'}.issubset(z.namelist())
print(f'Packaged {len(files)} files; {((paper/"paper_source.zip").stat().st_size/1024):.0f} KiB')
