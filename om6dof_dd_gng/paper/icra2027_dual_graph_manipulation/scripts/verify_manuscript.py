#!/usr/bin/env python3
"""Mechanical manuscript checks. These do not certify scientific readiness."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

PAPER = Path(__file__).resolve().parents[1]
def run(*args):
    return subprocess.check_output(args, cwd=PAPER, text=True)

info = run('pdfinfo', 'main.pdf')
text = run('pdftotext', 'main.pdf', '-')
fonts = run('pdffonts', 'main.pdf')
log = (PAPER/'main.log').read_text()
pages = int(re.search(r'^Pages:\s+(\d+)', info, re.M)[1])
missing = re.findall(r'(?:LaTeX Warning: (?:Citation|Reference).*undefined.*|There were undefined references)', log)
overfull = re.findall(r'Overfull \\[hv]box.*', log)
identity_tokens = ['kublab','om6dof','/home/','Anonymous Authors']
identity = {x: text.lower().count(x.lower()) for x in identity_tokens}
data = json.loads((PAPER/'data/workspace_summary.json').read_text())
frames = json.loads((PAPER/'data/frame_verification.json').read_text())
required = ['33,401','15,175','3,198','9.57','21.07','0.439608','0.007673']
numerical = {s: s in text for s in required}
checks = {
    'page_count': pages,
    'within_official_8_page_limit': pages <= 8,
    'page_size': re.search(r'^Page size:\s+(.+)', info, re.M)[1],
    'undefined_citations_or_references': missing,
    'overfull_boxes': overfull,
    'type3_fonts': bool(re.search(r'Type\s+3',fonts)),
    'identity_token_counts': identity,
    'reported_data_tokens_present': numerical,
    'dataset_grid_rows': data['grid_rows'],
    'dataset_selected_rows': data['selected_green_witnesses'],
    'independent_fk_witness_count': frames['witness_count'],
    'independent_fk_checks': frames['checks'],
    'pdf_sha256': hashlib.sha256((PAPER/'main.pdf').read_bytes()).hexdigest(),
    'scope': 'Mechanical checks only; human scientific and submission review remains necessary.',
    'scientific_status': 'Offline model evidence; integrated physical task results not established.',
}
(PAPER/'verification.json').write_text(json.dumps(checks,indent=2)+'\n')
assert pages <= 8, f'{pages} pages exceeds ICRA2027 limit'
assert not missing, missing
assert not overfull, overfull
assert not checks['type3_fonts'], 'Type3 fonts found'
assert identity['Anonymous Authors'] > 0
assert not any(identity[s] for s in ('kublab','om6dof','/home/')), identity
assert all(numerical.values()), numerical
assert data['grid_rows']==33401 and data['selected_green_witnesses']==3198
assert frames['witness_count']==3198 and all(frames['checks'].values())
print(f'Verified {pages} pages, resolved references, no overfull boxes, anonymous manuscript.')
